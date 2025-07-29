import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

class ApriltagY(Node):
    def __init__(self):
        super().__init__("apriltag_y_output")    # names the node when running

        self.kp = 50
        self.ki = 0.5
        self.kd = 20

        self.i = 0.0


        self.p_error = 0.0
        self.p_time = self.get_clock().now()



        self.pub = self.create_publisher(Float64, "/ApriltagY_control_output", 10)

        self.goal_sub = self.create_subscription(PoseArray, "/tags", self.y_control, 10)

        self.get_logger().info("initialized ApriltagY node")
    
    def y_control(self, msg):
        self.c_time = self.get_clock().now()
        self.d_time = (self.c_time - self.p_time).nanoseconds / 1e9  # seconds

        if msg.poses:
            # --- Compute yaw error from all detected poses ---
            y_mean = np.mean(pose.position.x for pose in msg.poses)
            c_error = y_mean

            # --- PID Control ---
            u_p = self.kp * c_error

            self.i += c_error * self.d_time  # Integral accumulation
            u_i = self.ki * self.i

            derivative = (c_error - self.p_error) / self.d_time if self.d_time > 0 else 0.0
            u_d = self.kd * derivative

            u = u_p + u_i + u_d
            u = np.clip(u, -60, 60)

            # --- Publish control signal ---
            m = Float64()
            m.data = u
            self.pub.publish(m)

            # --- Logging and updates ---
            self.get_logger().info(f"Tag R Error: {c_error:.3f} | R Force: {u:.3f}")

            self.p_error = c_error
            self.p_time = self.c_time

        else:
            # --- No tags detected: reset integral and error, stop motor ---
            self.i = 0.0
            self.p_error = 0.0

            m = Float64()
            m.data = 0.0
            self.pub.publish(m)

            self.get_logger().info("No tag detected — stopping control.")

        


    
    


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagY()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()