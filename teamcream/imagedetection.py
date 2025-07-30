#/home/eugene/auvc_ws/src/teamcream/teamcream/imagedetection.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from teamcream.lane_detection import cut_top_half, detect_lines, draw_lines, detect_lanes, draw_lanes, get_lane_center, recommend_direction, recommend_turn
import numpy as np
import cv2
import os
from std_msgs.msg import Float64,Int16


class ImageDetection(Node):
    def __init__(self):
        super().__init__("image_detection")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "/camera",
            self.getLane,
            10
        )

        self.y_pub = self.create_publisher(Float64, "/lane_error_y", 10)

        self.r_pub = self.create_publisher(Int16, "/lane_error_r", 10)


        self.save_dir = os.path.expanduser("~/auvc_ws/saved_images")

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.get_logger().info(f"Initialized subscriber node. Images will be saved to {self.save_dir}")

        
    def getLane(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().debug(f"Received image with frame_id: {msg.header.frame_id}")
            frame_id = msg.header.frame_id

            #original_path = os.path.join(self.save_dir, f"{frame_id}_original.jpg")
            #edges_path = os.path.join(self.save_dir, f"{frame_id}_edges.jpg")
            lines_path = os.path.join(self.save_dir, f"{frame_id}_lines.jpg")
            #lanes_path = os.path.join(self.save_dir, f"lanes_{frame_id}.jpg")
            #cv2.imwrite(original_path, cv_image)

            img = cut_top_half(cv_image)
            

            lines, edges= detect_lines(img,20,60,3,150,25)
            #cv2.imwrite(edges_path, edges)

            img_lines = draw_lines(img,lines)
            cv2.imwrite(lines_path, img_lines)

            lanes = detect_lanes(img,lines)

            mid_intercept, mid_slope, mid_angle = get_lane_center(lanes, img)
            height, width, channels = img.shape

            if mid_intercept is None or mid_slope is None or mid_angle is None:
                self.get_logger().warn("No valid lane center detected.")
                return

            if mid_slope >= 0:
                mid_angle = -mid_angle

            r_error = int(round(mid_angle))
            y_error = mid_intercept - width/2
            y_error = y_error/10**5

            r = Int16()
            r.data = r_error
            y = Float64()
            y.data = y_error
            
            self.r_pub.publish(r)
            self.y_pub.publish(y)

            #img_lane = draw_lanes(img,lanes)
            #cv2.imwrite(lanes_path, img_with_lines)


        except Exception as e:
            self.get_logger().error(f"Error processing or saving image: {str(e)}")

    def destroy_node(self):
        # 清理 OpenCV 窗口
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()