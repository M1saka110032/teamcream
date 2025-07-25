import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from sensor_msgs.msg import Image
from lane_detection import detect_lines, draw_lines, detect_lanes, draw_lanes, get_lane_center, recommend_direction, cut_top_half
import numpy as np
import cv2

class ImageDection(Node):
    def __init__(self):
        super().__init__("image_dection")    # names the node when running

        self.sub = self.create_subscription(
            Image,        # the message type
            "bluerov2/camera",    # the topic name,
            self.getLane,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized subscriber node")
    
    def getLane(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        img = cv2.imread(cv_image)
        img = cut_top_half(img)
        filtered_lines, edges= detect_lines(img,20,60,3,250,100)

        lanes = detect_lanes(img,filtered_lines)
        mid_intercept, mid_slope, mid_angle = get_lane_center(lanes,img)

        self.get_logger().info(recommend_direction(mid_intercept, mid_slope, mid_angle,img))

def main(args=None):
    rclpy.init(args=args)
    node = ImageDection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()