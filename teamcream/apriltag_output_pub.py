import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl

class ApriltagOutput(Node):
    def __init__(self):
        super().__init__("apriltag_output_pub")
        self.x_output = 0.0
        self.y_output = 0.0
        self.depth_control = 0.0
        self.heading_control = 0.0

        self.depth_sub = self.create_subscription(
            Float64, "/ApriltagZ_control_output", self.depth_callback, 10)

        self.heading_sub = self.create_subscription(
            Float64, "/ApriltagR_control_output", self.heading_callback, 10)

        self.pub = self.create_publisher(ManualControl, "/manual_control", 10)

        self.x_sub = self.create_subscription(Float64, "/ApriltagX_control_output", self.set_x_output_callback, 10)

        self.y_sub = self.create_subscription(Float64, "/ApriltagY_control_output", self.set_y_output_callback, 10)

        self.timer = self.create_timer(0.2, self.publish_manual_control)  # 5Hz


        self.get_logger().info("Initialized ApriltagOutput node")

    def depth_callback(self, msg):
        self.depth_control = msg.data

    def heading_callback(self, msg):
        self.heading_control = msg.data
    
    def set_x_output_callback(self, msg):
        new_x_output = msg.data
        self.x_output = new_x_output

    def set_y_output_callback(self, msg):
        new_y_output = msg.data
        self.y_output = new_y_output

    def publish_manual_control(self):
        msg = ManualControl()
        msg.x = float(self.x_output)
        msg.y = float(self.y_output)
        msg.z = float(self.depth_control)
        msg.r = float(self.heading_control)
        msg.buttons = 0
        self.pub.publish(msg)
        self.get_logger().info(f"x:{msg.x:.3f} y:{msg.y:.3f} z:{msg.z:.3f} r:{msg.r:.3f}")
def main(args=None):
    rclpy.init(args=args)
    node = ApriltagOutput()
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