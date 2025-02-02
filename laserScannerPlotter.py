import csv
import math
import matplotlib.pyplot as plt

# Define the file path
file_path = "laser_content_line.csv"  # Adjust if needed

# Open and read the CSV file from path
with open(file_path, "r") as file:
    reader = csv.reader(file)
    
    # Extract headers
    headers = next(reader)  # First row contains headers
    raw_data = next(reader) # Second row contains laser scan data

    angle_increment = float(raw_data[-2]) # Second last entry contains angle increment
    curr_angle = 0

    x_pos = []
    y_pos = []
    
    # Sort through list, 1st index and last 2 indexes are not needed
    for val in raw_data[1:-3]:
        if val == raw_data[1]:
            #for the first index remove the [ infront of the string
            val = val.replace("[","")

        #for all real values find the x and y cartesian coordinates
        if math.isfinite(float(val)):
            x_pos.append(float(val)*math.cos(curr_angle))
            y_pos.append(float(val)*math.sin(curr_angle))

        #incrrement the measure angle
        curr_angle += angle_increment

#Plot the cartisian coordinates
plt.title("Laser Scan Sensor Line Motion")
plt.xlabel("Distance from robot in x direction")
plt.ylabel("Distance from robot in y direction")
plt.grid()
plt.scatter(x_pos, y_pos, s=25)
plt.show()