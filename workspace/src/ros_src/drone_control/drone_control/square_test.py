#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class SquareMission(Node):

    def __init__(self):
        super().__init__('square_mission_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers & Publishers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_cb, qos_profile)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Clients (Note the corrected paths!)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Mission Waypoints [x, y, z]
        self.waypoints = [
            [0.0, 0.0, 2.0],  # 1. Hover at 2m
            [2.0, 0.0, 2.0],  # 2. Move East 2m
            [2.0, 2.0, 2.0],  # 3. Move North 2m
            [0.0, 2.0, 2.0],  # 4. Move West 2m
            [0.0, 0.0, 2.0]   # 5. Back to start
        ]
        self.current_wp_index = 0
        self.current_pos = None
        self.current_state = State()

        # Timer
        self.timer = self.create_timer(0.05, self.timer_cb) # 20Hz
        
        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True
        
        self.steps_counter = 0

    def state_cb(self, msg):
        self.current_state = msg

    def pos_cb(self, msg):
        self.current_pos = msg

    def distance_to_target(self, target_x, target_y, target_z):
        if self.current_pos is None:
            return 100.0 # Return huge distance if no position yet
        
        dx = target_x - self.current_pos.pose.position.x
        dy = target_y - self.current_pos.pose.position.y
        dz = target_z - self.current_pos.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def timer_cb(self):
        # 1. Safety Check
        if self.current_pos is None:
            return

        # 2. Publish target
        target = self.waypoints[self.current_wp_index]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target[0])
        pose.pose.position.y = float(target[1])
        pose.pose.position.z = float(target[2])
        self.local_pos_pub.publish(pose)

        # 3. Check State & Arming (Standard Offboard Logic)
        if self.current_state.mode != "OFFBOARD" and (self.get_clock().now().nanoseconds / 1e9) > 5.0:
            if self.steps_counter % 100 == 0: # Try every 5 seconds
                self.set_mode_client.call_async(self.offb_set_mode)
        elif not self.current_state.armed and self.current_state.mode == "OFFBOARD":
            if self.steps_counter % 100 == 0:
                self.arming_client.call_async(self.arm_cmd)
        
        # 4. Mission Logic: Did we reach the waypoint?
        dist = self.distance_to_target(target[0], target[1], target[2])
        
        if dist < 0.2: # 20cm tolerance
            if self.current_wp_index < len(self.waypoints) - 1:
                self.current_wp_index += 1
                self.get_logger().info(f'Reached WP! Moving to: {self.waypoints[self.current_wp_index]}')
            else:
                self.get_logger().info('Mission Complete. Hovering.')

        self.steps_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SquareMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()