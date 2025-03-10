# Imports

import sys
import threading
import rclpy

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController
from rclpy.executors import MultiThreadedExecutor

# You may add any other imports you may need/want to use below
# import ...
from collections import namedtuple
import math

class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(Twist,'/cmd_vel',10)
        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, kli=5, klv=0.1, kap=0.8, kai=2.5, kav=0.1)
            self.planner=planner(POINT_PLANNER)    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, kli=4, klv=0.1, kap=1.1, kai=2.5, kav=0.8)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)

        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner        
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)

    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.
        reached_goal = False

        #get current position
        rclpy.spin_once(self.localizer)
        
        curr_pose = self.localizer.getPose()
        # print(f"Current Pose: {curr_pose}")
        # goal_list = list(zip(self.goal[0], self.goal[1]))
        # print(f"goallist: {type(goal_list)}")
        
        if curr_pose  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()

        velocity, yaw_rate = self.controller.vel_request(curr_pose, self.goal, True)
        
        # TODO Part 3: Check if you reached the goal

        # Initialize self.goal_index if not set
        if not hasattr(self, "goal_index"):
            self.goal_index = 0  # Start from the first point

        if type(self.goal) == list: #aka is a path
            # print(self.goal)      
            current_goal=(self.goal[-1][0],self.goal[-1][1]) # needs modification before passing in
            # print(current_goal)
        else: #otherwise it will be a point
            current_goal=self.goal
            
        reached_goal = calculate_linear_error(curr_pose, current_goal) < 0.1

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            raise SystemExit()

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate

        self.publisher.publish(vel_msg) 
    

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        # DM=decision_maker(motion_type=POINT_PLANNER, goalPoint=[2.0, 2.0], publisher_msg=Twist, publishing_topic='/cmd_vel', qos_publisher=10)
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [1,1])
    elif args.motion.lower() == "trajectory":
        # DM=decision_maker(motion_type=TRAJECTORY_PLANNER, goalPoint=[-1.0, -1.0], publisher_msg=Twist, publishing_topic='/cmd_vel', qos_publisher=10)
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [0, 0], motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    while True:
        try:
            spin_once(DM)
            spin_once(DM.localizer)
        except SystemExit:
            print(f"reached there successfully {DM.localizer.pose}")
            break
        except KeyboardInterrupt:
            break


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
