import matplotlib.pyplot as plt
from utilities import FileReader
import pandas as pd
import numpy as np


def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()

    # time_list=[]
    
    # first_stamp=values[0][-1]
    
    # for val in values:
    #     time_list.append(val[-1] - first_stamp)

    
    
    # fig, axes = plt.subplots(1,2, figsize=(14,6))


    # axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    # axes[0].set_title("state space")
    # axes[0].grid()

    
    # axes[1].set_title("each individual state")
    # for i in range(0, len(headers) - 1):
    #     axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    # axes[1].legend()
    # axes[1].grid()

    # plt.show()

    robot_pose = pd.read_csv("robot_pose.csv")
    linear = pd.read_csv("linear.csv")
    angular = pd.read_csv("angular.csv")

    robot_pose.columns = robot_pose.columns.str.strip()
    linear.columns = linear.columns.str.strip()
    angular.columns = angular.columns.str.strip()

    # Convert timestamps to seconds (assuming nanosecond timestamps)
    robot_pose["stamp"] /= 1e9
    linear["stamp"] /= 1e9
    angular["stamp"] /= 1e9

    num = robot_pose["stamp"].iloc[0]
    for i, val in enumerate(robot_pose["stamp"]):
        robot_pose["stamp"].iloc[i] = robot_pose["stamp"].iloc[i] - num
    
    num = linear["stamp"].iloc[0]
    for i, val in enumerate(linear["stamp"]):
        linear["stamp"].iloc[i] = linear["stamp"].iloc[i] - num

    num = angular["stamp"].iloc[0]
    for i, val in enumerate(angular["stamp"]):
        angular["stamp"].iloc[i] = angular["stamp"].iloc[i] - num

    # Merge data on closest timestamps
    linear_merged = pd.merge_asof(linear.sort_values("stamp"), robot_pose.sort_values("stamp"), on="stamp")
    angular_merged = pd.merge_asof(angular.sort_values("stamp"), robot_pose.sort_values("stamp"), on="stamp")

    # Plot e vs t and e_dot vs t (Linear)
    plt.figure(figsize=(4, 5))
    plt.plot(linear_merged["stamp"], linear_merged["e"], label="e (linear)")
    plt.plot(linear_merged["stamp"], linear_merged["e_dot"], label="e_dot (linear)")
    plt.ylabel("Error (e) and Error Rate (edot)")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.title("Linear Error and Error Rate vs Time")
    plt.grid(True)
    plt.show()

    # Plot e vs t and e_dot vs t (Angular)
    plt.figure(figsize=(5, 5))
    plt.plot(angular_merged["stamp"], angular_merged["e"], label="e (angular)")
    plt.plot(angular_merged["stamp"], angular_merged["e_dot"], label="e_dot (angular)")
    plt.ylabel("Error (e) and Error Rate (edot)")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.title("Angular Error and Error Rate vs Time")
    plt.grid(True)
    plt.show()

    # Plot x vs t, y vs t, theta vs t
    plt.figure(figsize=(5, 5))
    plt.plot(robot_pose["stamp"], robot_pose["x"], label="x")
    plt.plot(robot_pose["stamp"], robot_pose["y"], label="y")
    plt.plot(robot_pose["stamp"], robot_pose["theta"], label="theta")
    plt.xlabel("Time (s)")
    plt.ylabel("State")
    plt.legend()
    plt.title("Position and Orientation vs Time")
    plt.grid(True)
    plt.show()

    # Plot x vs y
    plt.figure(figsize=(5, 5))
    plt.plot(robot_pose["x"], robot_pose["y"], label="Trajectory")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.title("Trajectory (x vs y)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    point = (1,1)
    # agility - rise time - subtract 10% from 90% to get agility
    distances = []
    for i in range(len(robot_pose)):
        val_x = robot_pose["x"].iloc[i]
        val_y = robot_pose["y"].iloc[i]
        distance = ((point[0]-val_x)**2 + (point[1]-val_y)**2)** 0.5
        distances.append(distance)

    real_dist = (point[0]**2 + point[1]**2)**0.5

    x = 0
    y = 0
    for i, val in enumerate(distances):
        if val/real_dist <= 0.9:
            x = robot_pose["stamp"][i]
            break
    
    for i, val in enumerate(distances):
        if val/real_dist <= 0.1:
            y = robot_pose["stamp"][i]
            break
    print(x)
    print(y)
    print(x-y)

    # accuracy
    true_x = np.linspace(0,point[1],len(robot_pose))
    true_y = true_x

    rmse = 0
    for i in range(len(robot_pose)):
        rmse += (abs(true_x[i] - robot_pose["x"].iloc[i])**2 + abs(true_x[i] - robot_pose["y"].iloc[i])**2)**0.5
    rmse = rmse/len(robot_pose)
    print(rmse)

    # Plot e vs e_dot (Linear)
    plt.figure(figsize=(5, 5))
    plt.plot(linear_merged["e"], linear_merged["e_dot"], 'o', label="Linear")
    plt.xlabel("Error (e)")
    plt.ylabel("Error Rate (e_dot)")
    plt.legend()
    plt.title("Linear Error vs Error Rate")
    plt.grid(True)
    plt.show()

    # Plot e vs e_dot (Angular)
    plt.figure(figsize=(5, 5))
    plt.plot(angular_merged["e"], angular_merged["e_dot"], 'o', label="Angular")
    plt.xlabel("Error (e)")
    plt.ylabel("Error Rate (e_dot)")
    plt.legend()
    plt.title("Angular Error vs Error Rate")
    plt.grid(True)
    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)



