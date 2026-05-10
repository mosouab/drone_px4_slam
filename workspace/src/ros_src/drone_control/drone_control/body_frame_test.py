#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class BodyFrameCircle(Node):
    # Type mask bits for PositionTarget
    IGNORE_PX = 1
    IGNORE_PY = 2
    IGNORE_PZ = 4
    IGNORE_VX = 8
    IGNORE_VY = 16
    IGNORE_VZ = 32
    IGNORE_AFX = 64
    IGNORE_AFY = 128
    IGNORE_AFZ = 256
    IGNORE_YAW = 1024
    IGNORE_YAW_RATE = 2048
    
    # Coordinate frames
    FRAME_LOCAL_NED = 1
    FRAME_BODY_NED = 8

    def __init__(self):
        super().__init__('body_frame_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos_profile
        )
        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.timer = self.create_timer(0.05, self.timer_cb)  # 20Hz
        self.setpoint_counter = 0
        self.mission_counter = 0
        self.offboard_requested = False
        self.arming_requested = False
        self.circle_started = False
        self.target_altitude = 5.0

    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, msg):
        self.current_pose = msg

    def create_position_target(self, coordinate_frame, type_mask):
        """Helper to create PositionTarget message with common setup"""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.coordinate_frame = coordinate_frame
        msg.type_mask = type_mask
        return msg

    def timer_cb(self):
        # Send initial hover setpoints before switching to OFFBOARD
        # (PX4 requires setpoint stream before accepting OFFBOARD mode)
        if self.setpoint_counter < 100:
            msg = self.create_position_target(
                self.FRAME_LOCAL_NED,
                self.IGNORE_VX | self.IGNORE_VY | self.IGNORE_VZ |
                self.IGNORE_AFX | self.IGNORE_AFY | self.IGNORE_AFZ | 
                self.IGNORE_YAW | self.IGNORE_YAW_RATE
            )
            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = 2.0  # Hover at 2m
            self.setpoint_pub.publish(msg)
            self.setpoint_counter += 1
            self.get_logger().info(f"Sending initial setpoints: {self.setpoint_counter}/100", throttle_duration_sec=1.0)
            return

        # Request OFFBOARD mode after sending enough setpoints
        if not self.offboard_requested and self.current_state.mode != "OFFBOARD":
            future = self.set_mode_client.call_async(SetMode.Request(custom_mode='OFFBOARD'))
            future.add_done_callback(self.mode_callback)
            self.offboard_requested = True
            self.get_logger().info("Requesting OFFBOARD mode...")
            return
        
        # Wait for OFFBOARD mode before arming
        if self.current_state.mode != "OFFBOARD":
            # Keep sending hover setpoint while waiting
            msg = self.create_position_target(
                self.FRAME_LOCAL_NED,
                self.IGNORE_VX | self.IGNORE_VY | self.IGNORE_VZ |
                self.IGNORE_AFX | self.IGNORE_AFY | self.IGNORE_AFZ | 
                self.IGNORE_YAW | self.IGNORE_YAW_RATE
            )
            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = 2.0
            self.setpoint_pub.publish(msg)
            self.get_logger().info("Waiting for OFFBOARD mode...", throttle_duration_sec=1.0)
            return

        # Request arming once in OFFBOARD mode
        if not self.arming_requested and not self.current_state.armed:
            future = self.arming_client.call_async(CommandBool.Request(value=True))
            future.add_done_callback(self.arming_callback)
            self.arming_requested = True
            self.get_logger().info("Requesting arming...")
            return

        # Wait for arming before executing mission
        if not self.current_state.armed:
            # Keep sending hover setpoint while waiting for arming
            msg = self.create_position_target(
                self.FRAME_LOCAL_NED,
                self.IGNORE_VX | self.IGNORE_VY | self.IGNORE_VZ |
                self.IGNORE_AFX | self.IGNORE_AFY | self.IGNORE_AFZ | 
                self.IGNORE_YAW | self.IGNORE_YAW_RATE
            )
            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = 2.0
            self.setpoint_pub.publish(msg)
            self.get_logger().info("Waiting for arming...", throttle_duration_sec=1.0)
            return

        # Mission execution starts here (only when armed and in OFFBOARD)
        current_alt = self.current_pose.pose.position.z
        
        # Check if we've reached target altitude (with hysteresis)
        if not self.circle_started and current_alt >= self.target_altitude - 0.3:
            self.circle_started = True
            self.get_logger().info(f"Reached target altitude, starting circle!")
        
        # Phase 1: Takeoff to target altitude using position control
        if not self.circle_started:
            msg = self.create_position_target(
                self.FRAME_LOCAL_NED,
                self.IGNORE_VX | self.IGNORE_VY | self.IGNORE_VZ |
                self.IGNORE_AFX | self.IGNORE_AFY | self.IGNORE_AFZ | 
                self.IGNORE_YAW | self.IGNORE_YAW_RATE
            )
            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = self.target_altitude
            self.get_logger().info(f"Takeoff: altitude={current_alt:.2f}m, target={self.target_altitude}m", throttle_duration_sec=0.5)
            
        # Phase 2: Circle using LOCAL_NED with velocity commands
        # This allows proper altitude hold while moving forward and yawing
        else:
            msg = self.create_position_target(
                self.FRAME_LOCAL_NED,
                self.IGNORE_PX | self.IGNORE_PY |  # Ignore X,Y position
                self.IGNORE_AFX | self.IGNORE_AFY | self.IGNORE_AFZ |  # Ignore acceleration
                self.IGNORE_YAW  # Ignore yaw angle, use yaw_rate
            )
            # Get current yaw from pose to calculate forward velocity in world frame
            import math
            q = self.current_pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            forward_speed = 2.0
            msg.velocity.x = forward_speed * math.cos(yaw)  # Forward in world frame
            msg.velocity.y = forward_speed * math.sin(yaw)  # Forward in world frame
            msg.velocity.z = 0.0
            msg.position.z = self.target_altitude  # Hold altitude with position
            msg.yaw_rate = 0.5  # Turn left
            self.get_logger().info(f"Circle: alt={current_alt:.2f}m, yaw={math.degrees(yaw):.1f}°", throttle_duration_sec=1.0)

        self.setpoint_pub.publish(msg)
        self.mission_counter += 1

    def mode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode set successfully")
            else:
                self.get_logger().warn("Failed to set OFFBOARD mode")
                self.offboard_requested = False  # Retry
        except Exception as e:
            self.get_logger().error(f"Mode service call failed: {e}")
            self.offboard_requested = False

    def arming_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed successfully")
            else:
                self.get_logger().warn("Failed to arm vehicle")
                self.arming_requested = False  # Retry
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")
            self.arming_requested = False

def main(args=None):
    rclpy.init(args=args)
    node = BodyFrameCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()