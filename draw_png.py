from math import cos, pi, sin
from matplotlib import patches, pyplot as plt
import matplotlib
import matplotlib.animation as animation
from numpy.core.fromnumeric import repeat
import pandas as pd
import numpy as np

filename = "records/normal_lawmix.txt"
figname = "figures/varmix"
data = np.loadtxt(filename, delimiter=',', skiprows= 1)

data = data[data[:,1]>35,:]

# lane_change_trajectory[:,0] += 195

data_bppolicy = data[data[:,5]==1,:]
data_rlpolicy = data[data[:,5]!=1,:]
data_rlpolicy_1 = data_rlpolicy[data_rlpolicy[:,1]>65,:]
data_rlpolicy_2 = data_rlpolicy[data_rlpolicy[:,1]<65,:]


fig, ax = plt.subplots(figsize = (2,7))
ax.set_xlim(195,215)
ax.set_ylim(30,100)
ax.set_xticklabels([])
# ax.set_xticks([])
ax.set_yticklabels([])


width = 4.5
height = 1.5
half_w = width/2.0
half_h = height/2.0

ego_line_1, = ax.plot([], [],'g')
ego_line_2, = ax.plot([], [],'g')
ego_line_bp, = ax.plot([], [],'r')
other_line, = ax.plot([], [],'b')

ego_line_1.set_data(data_rlpolicy_1[:,0], data_rlpolicy_1[:,1])
ego_line_2.set_data(data_rlpolicy_2[:,0], data_rlpolicy_2[:,1])
ego_line_bp.set_data(data_bppolicy[:,0],data_bppolicy[:,1])
other_line.set_data(data[:,3],data[:,4])


def get_bounds(x,y,yaw):
    rad = yaw*pi/180.0
    x0 = x - cos(rad)*half_w + sin(rad)*half_h
    y0 = y - sin(rad)*half_w - cos(rad)*half_h
    x1 = x + cos(rad)*half_w + sin(rad)*half_h
    y1 = y + sin(rad)*half_w - cos(rad)*half_h
    x2 = x + cos(rad)*half_w - sin(rad)*half_h
    y2 = y + sin(rad)*half_w + cos(rad)*half_h
    x3 = x - cos(rad)*half_w - sin(rad)*half_h
    y3 = y - sin(rad)*half_w + cos(rad)*half_h
    return np.array([[x0,y0], [x1, y1], [x2, y2], [x3, y3]])

check_point = ((data_rlpolicy_2[data_rlpolicy_2[:,0]>205.5,:])[0]).reshape(-1)
# check_point = data_rlpolicy_2[0]
# print(check_point)

ego_vehicle = patches.Polygon(xy = np.array([[0,0], [1,0], [1,1]]))
other_vehicle = patches.Polygon(xy = np.array([[0,0], [1,0], [1,1]]))

ego_vehicle.set_color('g')
other_vehicle.set_color('blue')
ego_vehicle.set_alpha(0.5)
other_vehicle.set_alpha(0.5)

ego_x = check_point[0]
ego_y = check_point[1]
ego_yaw = check_point[2]

other_x = check_point[3]
other_y = check_point[4]

ego_vehicle.set_xy(get_bounds(ego_x, ego_y, ego_yaw))
other_vehicle.set_xy(get_bounds(other_x, other_y, -90))

ax.add_patch(ego_vehicle)
ax.add_patch(other_vehicle)


fig.savefig("{}.eps".format(figname))
fig.savefig("{}.jpg".format(figname))
print("fix distance", other_y-ego_y)





# ego_vehicle = patches.Polygon(xy = np.array([[0,0], [1,0], [1,1]]))
# other_vehicle = patches.Polygon(xy = np.array([[0,0], [1,0], [1,1]]))