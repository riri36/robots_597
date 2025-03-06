from math import atan2, asin, sqrt
import math

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    # just unpack yaw (z-axis rotation)
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    # x, y, z, w = quat

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = atan2(siny_cosp, cosy_cosp)
    return yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
    
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.

    # Calculate the error vector x and y position
    
    # error_linear = math.dist([goal_pose[0], goal_pose[1]], [current_pose[0], current_pose[1]])
    x1, y1, _, _ = current_pose
    x2, y2 = goal_pose
    error_linear= sqrt(
        pow(y2 - y1, 2) +
        pow(x2 - x1, 2)
    )

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal

    # Calculate the error vectorx and y position
    # error_linear = [goal_pose[0] - current_pose[0] , goal_pose[1]-current_pose[1]]

    # curr_angle = current_pose[2]
    
    # # Use the linear error to find the desired angle we want the robot to face
    # goal_angle = atan2(error_linear[0],error_linear[1])

    # # Calculate both options for the robot to roatate to goal angle
    # angle_diff = [goal_angle - curr_angle, curr_angle - goal_angle]

    # # Take the smallest angle error and set that to error term
    # if abs(angle_diff[1]) > abs(angle_diff[0]):
    #     error_angular = angle_diff[0]
    # else:
    #     error_angular = angle_diff[1]

    # return error_angular

    x1, y1, theta1, _ = current_pose
    x2, y2 = goal_pose

    theta2 = atan2(
        y2 - y1,
        x2 - x1
    )

    error_angular = theta2 - theta1

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    while error_angular > M_PI:
        error_angular -= 2 * M_PI
    while error_angular < -M_PI:
        error_angular += 2 * M_PI

    return error_angular