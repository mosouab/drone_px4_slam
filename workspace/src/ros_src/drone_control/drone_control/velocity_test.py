#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class VelocityCircle(Node):

    def __init__(self):
        super().__init__('velocity_test_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_cb, qos_profile)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pos = PoseStamped()
        self.timer = self.create_timer(0.05, self.timer_cb) # 20Hz
        self.steps_counter = 0

    def state_cb(self, msg):
        self.current_state = msg

    def pos_cb(self, msg):
        self.current_pos = msg

    def get_yaw_from_quaternion(self, q):
        """
        Converts quaternion (w, x, y, z) to yaw angle in radians.
        """
        # Formula to convert quaternion to yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def timer_cb(self):
        vel_msg = Twist()
        
        current_z = self.current_pos.pose.position.z
        
        # Phase 1: Takeoff
        if current_z < 2.0:
            self.get_logger().info(f"Taking Off... Alt: {current_z:.2f}", throttle_duration_sec=1.0)
            vel_msg.linear.z = 1.0
        
        # Phase 2: Circle (The Fix)
        else:
            self.get_logger().info(f"Circling...", throttle_duration_sec=1.0)
            
            # 1. Get current Yaw
            q = self.current_pos.pose.orientation
            current_yaw = self.get_yaw_from_quaternion(q)

            # 2. Desired Forward Speed (Body Frame)
            forward_speed = 2.0

            # 3. Calculate Global Components             vel_msg.linear.x = forward_speed * math.cos(current_yaw)
            vel_msg.linear.y = forward_speed * math.sin(current_yaw)
            
            # 4. Set Yaw Rate (Spin)
            vel_msg.angular.z = 0.5
            vel_msg.linear.z = 0.0 # Hold altitude

        self.vel_pub.publish(vel_msg)

        # Standard Offboard Logic
        if self.current_state.mode != "OFFBOARD" and self.steps_counter % 20 == 0:
             self.set_mode_client.call_async(SetMode.Request(custom_mode='OFFBOARD'))
        if not self.current_state.armed and self.current_state.mode == "OFFBOARD" and self.steps_counter % 20 == 0:
            self.arming_client.call_async(CommandBool.Request(value=True))

        self.steps_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()