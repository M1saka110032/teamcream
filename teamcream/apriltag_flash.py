import rclpy    # the ROS 2 client library for Python
from rclpy.node import Node    # the ROS 2 Node class
from std_msgs.msg import Int16 , Float64    # the Int16 message type definition
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from mavros_msgs.msg import OverrideRCIn

class ApriltagFlash(Node):
    def __init__(self):
        super().__init__("apriltag_flash")    # names the node when running

        self.goal_sub = self.create_subscription(
            PoseArray, "/tags", self.tagdistance, 10)
        
        self.command_pub = self.create_publisher(OverrideRCIn, "override_rc", 10)

        self.get_logger().info(f"Initialized ApriltagFlash node.")


    def turn_lights_on(self, level):
        """
        Turn the lights on.
        Args:
            level (int): The level to turn the lights on to. 0 is off, 100 is full
        """
        self.get_logger().info(f"Turning lights on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        commands.channels[9] = 1000 + level * 10
        self.command_pub.publish(commands)

    def tagdistance(self, msg):
        distance = min(pose.position.z for pose in msg.poses)
        if distance < 1:
            self.turn_lights_on(100)
        else:
            self.turn_lights_on(0)


def main(args=None):
    rclpy.init(args=args)
    node = ApriltagFlash()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()