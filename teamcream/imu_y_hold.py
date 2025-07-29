import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float64
import numpy as np

class IMUYHold(Node):
    def __init__(self):
        super().__init__("imu_y_hold")

        self.kp = 50
        self.ki = 0.5
        self.kd = 20

        self.i = 0.0


        self.p_error = 0.0
        self.p_time = self.get_clock().now()

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',  # MAVROS topic
            self.imu_callback,
            10
        )

        self.pub = self.create_publisher(Float64, "/YHold_control_output", 10)

        self.get_logger().info("initialized IMU_YHold_control_output node")
    
    def imu_callback(self, msg):
        # 获取线性加速度 y 值
        accel_y = msg.linear_acceleration.y

        # 当前时间
        self.c_time = self.get_clock().now()
        self.d_time = (self.c_time - self.p_time).nanoseconds / 1e9

        if self.d_time <= 0.0:
            return

        if not hasattr(self, 'vel_y'):
            self.vel_y = 0.0
        self.vel_y += accel_y * self.d_time

        error = -self.vel_y 

        self.u_p = error * self.kp
        self.i += error * self.d_time
        self.u_i = self.ki * self.i
        derivative = (error - self.p_error) / self.d_time
        self.u_d = self.kd * derivative

        # 总输出
        self.u = self.u_p + self.u_i + self.u_d
        self.u = np.clip(self.u, -60, 60)

        # 发布控制信号
        m = Float64()
        m.data = self.u
        self.pub.publish(m)

        # 记录历史
        self.p_time = self.c_time
        self.p_error = error

        self.get_logger().info(f"Accel Y: {accel_y:.3f} | Vel Y: {self.vel_y:.3f} | Force: {self.u:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUYHold()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()