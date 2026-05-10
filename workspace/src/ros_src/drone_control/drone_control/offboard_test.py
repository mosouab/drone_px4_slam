#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control_node')

        # 1. Create Subscribers and Publishers
        
        # QoS profile to match MAVROS 'Best Effort' for telemetry if needed
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # State Subscriber (To know if we are connected/armed)
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10)
        
        # Local Position Publisher (Where we want the drone to go)
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10)

        # 2. Create Service Clients (To ask PX4 to do things)
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # 3. Wait for services to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for arming service...')
        
        self.current_state = State()
        
        # 4. Set the loop rate (20Hz is standard for MAVROS)
        self.timer = self.create_timer(0.05, self.timer_cb)
        
        self.offb_set_mode = SetMode.Request()
        self.arm_cmd = CommandBool.Request()

    def state_cb(self, msg):
        self.current_state = msg

    def timer_cb(self):
        # Create a pose message (0, 0, 2 meters high)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 500.0  # Hover height

        # Publish the setpoint
        self.local_pos_pub.publish(pose)

        # LOGIC: Ensure we are connected before trying anything
        if not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            return

        # LOGIC: Switch to OFFBOARD mode
        if self.current_state.mode != "OFFBOARD":
            # We only try to switch if 5 seconds have passed (simulated by simple counter logic or just retrying)
            # Here we just try to switch if we aren't in offboard
            self.offb_set_mode.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(self.offb_set_mode)
            self.get_logger().info('OFFBOARD enable request sent')

        # LOGIC: Arm the drone
        elif not self.current_state.armed:
            self.arm_cmd.value = True
            self.arming_client.call_async(self.arm_cmd)
            self.get_logger().info('Arming request sent')

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()