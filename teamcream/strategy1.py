import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl
import numpy as np
class Strategy1Pub(Node):
    def __init__(self):
        super().__init__("strategy1")
        self.max_power = 70
        
        self.x_output = 50
        self.y_output = 0.0
        self.depth_control = 0.0
        self.heading_control = 0.0

        self.apriltag_x = 0.0
        self.apriltag_y = 0.0
        self.apriltag_z = 0.0
        self.apriltag_r = 0.0
        self.lane_y = 0.0
        self.lane_r = 0.0
        self.depth = 0.0
        self.heading = 0.0

        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)

        self.depth_sub = self.create_subscription(
            Float64, "/depth_control_output", self.depth_callback, 10)
        self.heading_sub = self.create_subscription(
            Float64, "/heading_control_output", self.heading_callback, 10)
        self.lane_y_sub = self.create_subscription(
            Float64, "/laneY_control_output", self.lane_y_callback, 10)
        self.lane_r_sub = self.create_subscription(
            Float64, "/laneR_control_output", self.lane_r_callback, 10)
        self.apriltag_r_sub = self.create_subscription(
            Float64, "/ApriltagR_control_output", self.apriltag_r_callback, 10)
        self.apriltag_z_sub = self.create_subscription(
            Float64, "/ApriltagZ_control_output", self.apriltag_z_callback, 10)
        self.apriltag_x_sub = self.create_subscription(
            Float64, "/ApriltagX_control_output", self.apriltag_x_callback, 10)
        self.apriltag_y_sub = self.create_subscription(
            Float64, "/ApriltagY_control_output", self.apriltag_y_callback, 10)

        # 定时器，10Hz
        self.timer = self.create_timer(0.1, self.publish_manual_control)
        self.get_logger().info("Initialized Strategy1Pub node.")

    def depth_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid depth control value: {msg.data}")
            return
        self.depth = msg.data

    def heading_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid heading control value: {msg.data}")
            return
        self.heading = msg.data

    def lane_y_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid laneY control value: {msg.data}")
            return
        self.lane_y = msg.data

    def lane_r_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid laneR control value: {msg.data}")
            return
        self.lane_r = msg.data

    def apriltag_x_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid ApriltagX control value: {msg.data}")
            return
        self.apriltag_x = msg.data

    def apriltag_y_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid ApriltagY control value: {msg.data}")
            return
        self.apriltag_y = msg.data

    def apriltag_z_callback(self, msg):
        if not (-self.max_power <= msg.data <=self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid ApriltagZ control value: {msg.data}")
            return
        self.apriltag_z = msg.data

    def apriltag_r_callback(self, msg):
        if not (-self.max_power <= msg.data <= self.max_power) or np.isnan(msg.data):
            self.get_logger().warn(f"Invalid ApriltagR control value: {msg.data}")
            return
        self.apriltag_r = msg.data

    def publish_manual_control(self):
        msg = ManualControl()
        # 检查Apriltag输入是否有效（任意一个非0）
        apriltag_active = (
            abs(self.apriltag_x) > 0.0 or
            abs(self.apriltag_y) > 0.0 or
            abs(self.apriltag_z) > 0.0 or
            abs(self.apriltag_r) > 0.0
        )
        lane_active = abs(self.lane_y) > 0.0 or abs(self.lane_r) > 0.0

        # x_output逻辑：ApriltagX非0时使用，否则默认40
        self.x_output = self.apriltag_x if abs(self.apriltag_x) > 0.0 else 40.0
        x_source = "ApriltagX" if abs(self.apriltag_x) > 0.0 else "default"

        # y_output逻辑
        if apriltag_active:
            self.y_output = self.apriltag_y
            y_source = "ApriltagY"
        elif lane_active:
            self.y_output = self.lane_y
            y_source = "laneY"
        else:
            self.y_output = 0.0
            y_source = "none"

        # depth_control逻辑
        if apriltag_active:
            self.depth_control = self.apriltag_z
            z_source = "ApriltagZ"
        else:
            self.depth_control = self.depth
            z_source = "depth_control_output"

        # heading_control逻辑
        if apriltag_active:
            self.heading_control = self.apriltag_r
            r_source = "ApriltagR"
        elif lane_active:
            self.heading_control = self.lane_r
            r_source = "laneR"
        else:
            self.heading_control = self.heading
            r_source = "heading_control_output"

        # 发布消息
        msg.x = float(self.x_output)
        msg.y = float(self.y_output)
        msg.z = float(self.depth_control)
        msg.r = float(self.heading_control)
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info(
            f"Published ManualControl: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, r={msg.r:.2f}, "
            f"sources: x={x_source}, y={y_source}, z={z_source}, r={r_source}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Strategy1Pub()
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