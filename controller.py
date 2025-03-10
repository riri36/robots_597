import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        CONT_TYPE = P

        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(CONT_TYPE, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(CONT_TYPE, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)
        print(pose)

        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity

        linear_vel = 0.22 if linear_vel > 0.22 else linear_vel #0.31 in lab, 0.22 in sim
        angular_vel= 2.84 if angular_vel > 2.84 else angular_vel #1.90 in lab, 2.84 in sim
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        finalGoal=listGoals[-1]
        
        # print(finalGoal)
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)
        
        print(pose)

        timestamp = pose[3]
        
        linear_vel=self.PID_linear.update([e_lin, timestamp], status)
        angular_vel=self.PID_angular.update([e_ang, timestamp], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity

        linear_vel = 0.22 if linear_vel > 0.22 else linear_vel #0.31 in lab, 0.22 in sim
        angular_vel= 2.84 if angular_vel > 2.84 else angular_vel #1.90 in lab, 2.84 in sim
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 1, len(listGoals) - 1) ]
