#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
import cv2
import numpy as np
from my_py_pkg.vision_utils import (  # Updated import with package prefix
    compute_homography,
    recover_from_homography,
    estimate_homography,
    rodriguez,
    pbvs_controller
)

class PBVSNode(Node):
    def __init__(self):
        super().__init__('pbvs_node')
        
        # Declare parameters with correct default path
        self.declare_parameter('reference_image', 
                             '/home/c/ros2_ws/src/my_cpp_pkg/image_reference/reference_image.jpg')
        self.declare_parameter('lambdav', 0.5)
        self.declare_parameter('lambdaw', 0.5)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load reference image with error handling
        ref_path = self.get_parameter('reference_image').value
        self.ref_image = cv2.imread(ref_path, cv2.IMREAD_COLOR)
        if self.ref_image is None:
            self.get_logger().error(f"Failed to load reference image: {ref_path}")
            raise RuntimeError("Reference image not found")
            
        # Camera calibration matrix (should be parameterized)
        self.K = np.array([
            [554.382713, 0.0, 320.0],
            [0.0, 554.382713, 240.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Initialize state variables
        self.current_image = None
        self.H = np.eye(3)
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))
        self.n = np.zeros(3)
        self.d = 0.0
        self.counter = 0
        self.homography_solution = None
        
        # Setup QoS for image transport (ROS2-compliant)
        image_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/iris/usb_cam/image_raw',  # Verify correct topic name
            self.image_callback,
            image_qos
        )
        
        # Create publishers with appropriate QoS
        self.homography_pub = self.create_publisher(Float64MultiArray, '/homography_numerical', 10)
        self.ck_t_ct_pub = self.create_publisher(Float64MultiArray, '/ck_t_ct', 10)
        self.n_pub = self.create_publisher(Float64MultiArray, '/n_plane_vector', 10)
        self.Uv_pub = self.create_publisher(Float64MultiArray, '/Uv', 10)
        self.Uw_pub = self.create_publisher(Float64MultiArray, '/Uw', 10)
        self.d_pub = self.create_publisher(Float64, '/d_value', 10)
        
        # Processing timer (10Hz)
        self.process_timer = self.create_timer(0.1, self.process_callback)

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {str(e)}')

    def process_callback(self):
        if self.current_image is None:
            return
            
        try:
            # Estimate homography
            self.H = estimate_homography(
                self.ref_image, 
                self.current_image, 
                self.K, 
                self.counter
            )
            
            if self.H is not None:
                # Recover pose from homography
                (self.R, self.t, self.n, self.d, 
                 self.homography_solution) = recover_from_homography(
                    self.H, 
                    self.K, 
                    self.counter, 
                    self.homography_solution
                )
                
                # Compute control vectors
                u = rodriguez(self.R)
                lambdav = self.get_parameter('lambdav').value
                lambdaw = self.get_parameter('lambdaw').value
                Uv, Uw = pbvs_controller(self.R, self.t, u, lambdav, lambdaw)
                
                # Publish results
                self.publish_matrix(self.homography_pub, self.H)
                self.publish_matrix(self.ck_t_ct_pub, self.t)
                self.publish_matrix(self.n_pub, self.n)
                self.d_pub.publish(Float64(data=self.d))
                self.publish_matrix(self.Uv_pub, Uv)
                self.publish_matrix(self.Uw_pub, Uw)
                
            self.counter += 1
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}', throttle_duration_sec=1.0)

    def publish_matrix(self, publisher, matrix):
        """Helper function to publish numpy matrices as Float64MultiArray"""
        msg = Float64MultiArray()
        msg.data = matrix.flatten().tolist()
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PBVSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Node initialization failed: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
