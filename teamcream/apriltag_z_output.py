import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

class ApriltagZ(Node):
    def __init__(self):
        super().__init__("apriltag_z_output")    # names the node when running

        self.kp = 50
        self.ki = 0.5
        self.kd = 20

        self.i = 0.0


        self.p_error = 0.0
        self.p_time = self.get_clock().now()



        self.pub = self.create_publisher(Float64, "/ApriltagZ_control_output", 10)

        self.goal_sub = self.create_subscription(PoseArray, "/tags", self.z_control, 10)

        self.get_logger().info("initialized ApriltagZ node")
    
    def z_control(self, msg):
        z_mean = np.mean(pose.position.y for pose in msg.poses)

        self.c_error = z_mean
        self.c_time = self.get_clock().now()
        self.d_time = (self.c_time-self.p_time).nanoseconds/10**9

        self.u_p = self.c_error * self.kp

        self.i += self.d_time * self.c_error
        self.u_i = self.ki * self.i

        if self.d_time > 0:
            derivative = (self.c_error - self.p_error) / self.d_time
        else:
            derivative = 0.0
        self.u_d = self.kd * derivative

        self.u = (self.u_p + self.u_i + self.u_d)

        self.u = np.clip(self.u, -60, 60)

        m = Float64()
        m.data = self.u
        self.pub.publish(m)

        self.p_time = self.c_time
        self.p_error = self.c_error

        self.get_logger().info(f"Tag Z Error: {self.c_error:.2f} Tag Z Force: {self.u:.2f}")


        


    
    


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagZ()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()