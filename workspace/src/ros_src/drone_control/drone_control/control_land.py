#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Vector3, PoseStamped

class ControlLand(Node):

    def __init__(self):
        super().__init__('control_land_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.target_sub = self.create_subscription(Vector3, '/landing/target', self.target_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile)
        
        # Publishers
        self.raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.last_target = Vector3()
        self.last_target.z = 0.0 # Start as lost
        self.current_altitude = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_altitude = 2.0  # Default hover altitude; overridden by 'target_altitude' ROS param
        self.declare_parameter('target_altitude', self.target_altitude)
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value
        self.takeoff_complete = False
        self.landing_initiated = False  # Flag to stop control after landing
        
        # We need a timer to keep publishing setpoints even if vision is slow
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.steps_counter = 0

    def state_cb(self, msg):
        self.current_state = msg

    def target_cb(self, msg):
        self.last_target = msg
        # Debug: Log when we receive target messages
        if msg.z == 1.0:
            self.get_logger().info(f"[TARGET RX] Marker visible: X={msg.x:.3f}, Y={msg.y:.3f}", throttle_duration_sec=1.0)
        else:
            self.get_logger().info("[TARGET RX] Marker LOST", throttle_duration_sec=2.0)

    def pose_cb(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_altitude = msg.pose.position.z
        # Check if takeoff is complete
        if self.current_altitude >= self.target_altitude * 0.95:
            self.takeoff_complete = True

    def control_loop(self):
        # Stop control loop if landing has been initiated
        if self.landing_initiated:
            return
        
        # LOGIC:
        # Phase 1: Takeoff to target altitude using POSITION setpoints (more reliable)
        if not self.takeoff_complete:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = self.current_x
            pose_msg.pose.position.y = self.current_y
            pose_msg.pose.position.z = self.target_altitude  # Go to 2m altitude
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
            self.get_logger().info(f"Taking off... Alt: {self.current_altitude:.2f}m / {self.target_altitude}m")
        
        # Phase 2 & 3: Tracking/Landing logic (after takeoff complete)
        else:
            msg = PositionTarget()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.coordinate_frame = 1  # LOCAL_NED frame
            msg.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 1024 # Velocity Only

            if self.last_target.z == 1.0:
                # VISIBLE: Track it
                # Map Image Coordinates to Drone Body Frame
                # Target error: positive = marker is right/down in image
                # We want to move OPPOSITE to the error to center the marker
                # Body Frame NED: X=forward, Y=right, Z=down
                
                # Invert signs: move opposite to error
                vel_y = -0.5 * self.last_target.x  # Marker right -> move left
                vel_x = -0.5 * self.last_target.y  # Marker down -> move forward
                
                # Auto-land when close to ground
                if self.current_altitude < 0.5:
                    self.get_logger().info("=== CLOSE TO GROUND - SWITCHING TO AUTO.LAND ===")
                    self.landing_initiated = True
                    self.set_mode_client.call_async(SetMode.Request(custom_mode='AUTO.LAND'))
                    return
                
                # Descent Logic (NED: negative Z = down)
                if abs(self.last_target.x) < 0.1 and abs(self.last_target.y) < 0.1:
                    vel_z = -0.3  # Go Down (NEGATIVE Z is down in NED frame)
                    self.get_logger().info(f"Descending... Alt: {self.current_altitude:.2f}m")
                else:
                    vel_z = 0.0
                    self.get_logger().info(f"Aligning... Err X: {self.last_target.x:.2f}, Y: {self.last_target.y:.2f} | Vel X: {vel_x:.2f}, Y: {vel_y:.2f}")

                msg.velocity.x = vel_x
                msg.velocity.y = vel_y
                msg.velocity.z = vel_z
                
            else:
                # LOST: Hover in place
                msg.velocity.x = 0.0
                msg.velocity.y = 0.0
                msg.velocity.z = 0.0
                self.get_logger().info(f"Hovering... Alt: {self.current_altitude:.2f}m")

            self.raw_pub.publish(msg)
            
        # Standard Offboard/Arming Logic (Auto-start)
        if self.current_state.mode != "OFFBOARD" and self.steps_counter % 20 == 0:
             self.set_mode_client.call_async(SetMode.Request(custom_mode='OFFBOARD'))
             
        if not self.current_state.armed and self.current_state.mode == "OFFBOARD" and self.steps_counter % 20 == 0:
            self.arming_client.call_async(CommandBool.Request(value=True))

        self.steps_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControlLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()