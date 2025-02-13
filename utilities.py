from math import atan2, asin, sqrt

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
    error_linear = [goal_pose[0][0] - current_pose[0] , goal_pose[0][1]-current_pose[1]]

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal

    # Calculate the error vectorx and y position
    error_linear = [goal_pose.x - current_pose.x , goal_pose.y-current_pose.y]

    # #Assumes theta is given from 0 to 2pi
    # curr_angle = current_pose.theta
    # if curr_angle > M_PI:
    #     curr_angle = -1*(curr_angle-M_PI)

    #Assumes theta is give from pi to -pi
    curr_angle = current_pose.theta
    
    # Use the linear error to find the desired angle we want the robot to face
    goal_angle = atan2(error_linear[0],error_linear[1])

    # Calculate both options for the robot to roatate to goal angle
    angle_diff = [goal_angle - curr_angle, curr_angle - goal_angle]

    # Take the smallest angle error and set that to error term
    if abs(angle_diff[1]) > abs(angle_diff[0]):
        error_angular = angle_diff[0]
    else:
        error_angular = angle_diff[1]

    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    
    return error_angular
