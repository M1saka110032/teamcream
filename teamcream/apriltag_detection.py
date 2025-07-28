import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
from dt_apriltags import Detector
import os
import cv2
class AprilTagDetection(Node):
    def __init__(self):
        super().__init__("apriltag_detection")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "/camera",
            self.DetectAprilTags,
            10
        )

        self.pub = self.create_publisher(PoseArray, "/tags", 10)

        self.detector = Detector(families='tag36h11',
                                 nthreads=1,
                                 quad_decimate=1.0,
                                 quad_sigma=0.0,
                                 refine_edges=1,
                                 decode_sharpening=0.25,
                                 debug=0)
        
        self.save_dir = os.path.expanduser("~/auvc_ws/saved_images")

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        self.get_logger().info(f"Initialized subscriber node. Images will be saved to {self.save_dir}")

    def DetectAprilTags(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Error processing or saving image: {str(e)}")
        
        height, width, channels = cv_image.shape
        
        tags = self.detector.detect(gray_image,
                                     estimate_tag_pose=True,
                                     camera_params=(width,height,width/2,height/2),
                                     tag_size=0.1) #m
        
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "camera_frame"  #unknow

        if tags:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(cv_image, tuple(tag.corners[idx - 1, :].astype(int)),
                             tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

                cv2.putText(cv_image, str(tag.tag_id),
                            org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,
                            color=(0, 0, 255))
    

                pose_t = tag.pose_t
                x, y, z = pose_t[0][0], pose_t[1][0], pose_t[2][0]

                pose_R = np.array(tag.pose_R)
                r11 = pose_R[0, 0]
                r21 = pose_R[1, 0]

                yaw_rad = np.arctan2(r21, r11)

                pose_msg = Pose()
                pose_msg.position.x = x - 0.5 * z
                pose_msg.position.y = -(y - 0.5 * z)
                pose_msg.position.z = z - 0.5 #attack distance 1m

                pose_msg.orientation.x = 0.0
                pose_msg.orientation.y = 0.0
                pose_msg.orientation.z = np.sin(yaw_rad / 2)
                pose_msg.orientation.w = np.cos(yaw_rad / 2)

                pose_array_msg.poses.append(pose_msg)

                self.get_logger().info(
                    f"Tag ID {tag.tag_id}, Position: ({x:.2f}, {y:.2f}, {z:.2f}) m, Yaw: {np.degrees(yaw_rad):.2f}°"
                )
        else:
            self.get_logger().info("No tags detected.")

        self.pub.publish(pose_array_msg)

        timestamp = self.get_clock().now().to_msg()
        filename = f"tag_{timestamp.sec}.jpg"
        save_path = os.path.join(self.save_dir, filename)
        cv2.imwrite(save_path, cv_image)

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