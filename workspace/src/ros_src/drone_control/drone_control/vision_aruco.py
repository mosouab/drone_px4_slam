#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class VisionAruco(Node):

    def __init__(self):
        super().__init__('vision_aruco_node')

        # Publishers & Subscribers
        # We publish the error: X, Y, and Z (where Z is a flag: 1=Found, 0=Lost)
        self.target_pub = self.create_publisher(Vector3, '/landing/target', 10)
        
        # Camera topic — set via 'camera_topic' ROS parameter (default: /camera/image_raw)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.img_sub = self.create_subscription(Image, camera_topic, self.img_cb, 10)
        self.get_logger().info(f"Subscribing to camera topic: {camera_topic}")
        
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        
        # Create detector for newer OpenCV versions
        try:
            self.aruco_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
            self.get_logger().info("Using new ArUco API (OpenCV 4.7+)")
        except AttributeError:
            # For legacy API, use the old DetectorParameters creation method
            try:
                self.aruco_params = aruco.DetectorParameters_create()
            except:
                self.aruco_params = aruco.DetectorParameters()
            self.use_new_api = False
            self.get_logger().info("Using legacy ArUco API")
        
        self.frame_count = 0
        self.enable_visualization = False  # Headless on Pi 4 — no display available
        self.get_logger().info("Vision ArUco node started. Waiting for images on /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image...")

    def img_cb(self, msg):
        try:
            self.frame_count += 1
            
            # Log every 30 frames to show we're receiving images
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"Received frame #{self.frame_count}")
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f"Failed to convert image: {e}")
                return

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Use appropriate API based on OpenCV version
            if self.use_new_api:
                corners, ids, rejected = self.detector.detectMarkers(gray)
            else:
                corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            target_msg = Vector3()
            
            if ids is not None:
                # Marker Found
                # Calculate Center
                c = corners[0][0]
                cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
                cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4

                h, w = gray.shape
                
                # Normalize Error (-1.0 to 1.0)
                # x > 0 means Marker is to the Right
                # y > 0 means Marker is Down (Bottom of image)
                target_msg.x = (cx - (w / 2)) / (w / 2)
                target_msg.y = (cy - (h / 2)) / (h / 2)
                target_msg.z = 1.0 # Flag: VISIBLE
                
                self.get_logger().info(f"ArUco FOUND! ID: {ids[0][0]}, Error X: {target_msg.x:.3f}, Y: {target_msg.y:.3f}")

                # Visualization (only if enabled)
                if self.enable_visualization:
                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    cv2.circle(cv_image, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                    cv2.line(cv_image, (int(cx), int(cy)), (int(w/2), int(h/2)), (0, 0, 255), 2)
                
            else:
                # Marker Lost
                target_msg.x = 0.0
                target_msg.y = 0.0
                target_msg.z = 0.0 # Flag: LOST

            self.target_pub.publish(target_msg)
            
            # Only display if visualization is enabled (requires display server)
            if self.enable_visualization:
                try:
                    cv2.imshow("Vision Node View", cv_image)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warn(f"Display error: {e}. Disabling visualization.")
                    self.enable_visualization = False
        
        except Exception as e:
            self.get_logger().error(f"Critical error in image callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = VisionAruco()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()