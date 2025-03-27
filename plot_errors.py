import matplotlib.pyplot as plt
import pandas as pd
# from utilities import FileReader


df = pd.read_csv('robotPose.csv')
x = df["odom_x"]
y = df[" odom_y"]
th = df[" odom_th"]

pf_x = df[" pf_x"]
pf_y = df[" pf_y"]
pf_th = df[" pf_th"]

stamp = df[" stamp"]

plt.figure()
plt.plot(x,y, label="odom")
plt.plot(pf_x, pf_y, label="pf")
plt.title("Logged Positions x,y of PF and Odom")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(stamp,th, label="odom th")
plt.plot(stamp,pf_th, label="pf th")
plt.title("Logged Positions th of PF and Odom")
plt.xlabel("time")
plt.ylabel("theta")
plt.legend()
plt.grid()
plt.show()

# def plot_errors(filename):
    
#     headers, values=FileReader(filename).read_file()
    
#     time_list=[]
    
#     first_stamp=values[0][-1]
    
#     for val in values:
#         time_list.append(val[-1] - first_stamp)

    
    
#     fig, axes = plt.subplots(1,2, figsize=(14,6))


#     axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
#     axes[0].set_title("state space")
#     axes[0].grid()

    
#     axes[1].set_title("each individual state")
#     for i in range(0, len(headers) - 1):
#         axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

#     axes[1].legend()
#     axes[1].grid()

#     plt.show()
    
    





# import argparse

# if __name__=="__main__":

#     parser = argparse.ArgumentParser(description='Process some files.')
#     parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
#     args = parser.parse_args()
    
#     print("plotting the files", args.files)

#     filenames=args.files
#     for filename in filenames:
#         plot_errors(filename)



