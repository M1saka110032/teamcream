import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lane_detection import cut_top_half, detect_lines, draw_lines, detect_lanes, draw_lanes, get_lane_center, recommend_direction, recommend_turn
import numpy as np
import cv2
import os
from std_msgs.msg import Float64,Int16
from dt_apriltags import Detector

class AprilTagDetection(Node):
    def __init__(self):
        super().__init__("apriltag_detection")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "bluerov2/camera",
            self.getLane,
            10
        )

        self.save_dir = "saved_images"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.detector = Detector(families='tag36h11',
                                 nthreads=1,
                                 quad_decimate=1.0,
                                 quad_sigma=0.0,
                                 refine_edges=1,
                                 decode_sharpening=0.25,
                                 debug=0)
        
        self.get_logger().info(f"Initialized subscriber node. Images will be saved to {self.save_dir}")

    def getLane(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error processing or saving image: {str(e)}")
        
        height, width, channels = cv_image.shape

        tags = self.detector.detect(cv_image,
                                     estimate_tag_pose=True,
                                     camera_params=(width,height,width/2,height/2),
                                     tag_size=0.1) #m

        # 输出检测结果
        if tags:
            for tag in tags:
                pose = tag.pose_t
                distance = np.linalg.norm(pose)
                self.get_logger().info(f"Tag ID {tag.tag_id} is {distance:.2f} meters away,at center {tag.center}")
        else:
            self.get_logger().info("No tags detected.")
    def destroy_node(self):
        # 清理 OpenCV 窗口
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetection()
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