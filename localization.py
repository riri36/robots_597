import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        # check if right
        odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.create_subscription(odom, '/odom', self.odom_callback, qos_profile=odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg: odom):
        
        # TODO Part 3: Read x,y, theta, and record the stamp
        self.odom_initialized=True

        timestamp = pose_msg.header.stamp
        odom_orientation = pose_msg.pose.pose.orientation
        odom_pos = pose_msg.pose.pose.position
        # odom_lin_vel = pose_msg.twist.twist.linear
        # odom_angular_vel = pose_msg.twist.twist.angular
        
        yaw = euler_from_quaternion(odom_orientation)
        
        self.pose=[odom_pos.x, odom_pos.y ,yaw , timestamp]
        
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3

# TODO LAB 2

# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
    
if __name__ == "__main__":
    init(args=sys.argv)  # Initialize ROS 2

    node = localization()
    
    try:
        spin(node)  # Keep the node running
    except KeyboardInterrupt:
        print("Shutting down localization node.")
    finally:
        node.destroy_node()
