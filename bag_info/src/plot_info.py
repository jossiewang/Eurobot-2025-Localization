#!/usr/bin/env python3
import tf.transformations as tf
import matplotlib.pyplot as plt
import csv


# rostopic echo -p --bag=2021-07-07-15-00-00.bag --noarr /robot/pose >> lidar_bbb-sim-2.csv
# install tkinter to run the code: sudo apt install python3-tk
# change the y label and title to specify the plotting data
# csv file name structure: [listened topic]-[compensation option]-[from which bag]
# png file name structure: [listened topic]-[compensation option]-[plotting data]-[from which bag]

sim_file=open("lidar_bbb-sim-2.csv", mode="r", newline="")
tf_file=open("lidar_bbb-tf-2.csv", mode="r", newline="")
raw_file=open("lidar_bbb-raw-2.csv", mode="r", newline="")

sim_reader=csv.reader(sim_file)
tf_reader=csv.reader(tf_file)
raw_reader=csv.reader(raw_file)

sim_header=next(sim_reader)
tf_header=next(tf_reader)
raw_header=next(raw_reader)

sim_x=[]
sim_y=[]

for row in sim_reader:
    sim_x_val=(float(row[0])/float(1000000000))-float(1714581008.192987)
    _, _, yaw=tf.euler_from_quaternion([row[7], row[8], row[9], row[10]])
    sim_x.append(sim_x_val)
    # sim_y.append(float(row[4])) # x value
    # sim_y.append(float(row[5])) # y value
    # sim_y.append(yaw) # theta value

tf_x=[]
tf_y=[]

for row in tf_reader:
    tf_x_val=(float(row[0])/float(1000000000))-float(1714581008.193199)
    _, _, yaw=tf.euler_from_quaternion([row[7], row[8], row[9], row[10]])
    # tf_y.append(float(row[4])) # x value
    # tf_y.append(float(row[5])) # y value
    # tf_y.append(yaw) # theta value

raw_x=[]
raw_y=[]

for row in raw_reader:
    raw_x_val=(float(row[0])/float(1000000000))-float(1714581008.193199)
    _, _, yaw=tf.euler_from_quaternion([row[7], row[8], row[9], row[10]])
    raw_x.append(raw_x_val)
    # raw_y.append(float(row[4])) # x value
    # raw_y.append(float(row[5])) # y value
    # raw_y.append(yaw) # theta value

plt.plot(sim_x, sim_y, label="simple compensation")
plt.plot(tf_x, tf_y, label="TF commpensation")
plt.plot(raw_x, raw_y, label="raw data")

plt.xlabel('t')
plt.ylabel('y')
plt.legend()
plt.title('lidar_bonbonbon simple vs tf vs raw -- y')

plt.show()
# plt.savefig('lidar_bbb-3vs-x-2.png', bbox_inches='tight')
