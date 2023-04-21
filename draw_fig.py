from math import cos, pi, sin
from matplotlib import patches, pyplot as plt
import matplotlib
import matplotlib.animation as animation
import pandas as pd
import numpy as np

global FRAME_CUT 
FRAME_CUT = 1 # a frame is generated every FRAME_CUT frames. must be set as 1 in this code.
global BEGINOMIT
BEGINOMIT = 10
global last_law 
last_law = 0

fig, ax = plt.subplots(figsize = (2,7))
ax.set_xlim(195,215)
ax.set_ylim(30,100)
filename = "records/acc_law.txt"
gif_name = "figures/acc_with_law_fix.gif"

global start_lane_change
start_lane_change = False

width = 4.5
height = 1.5
half_w = width/2.0
half_h = height/2.0

ego_line, = ax.plot([], [])
other_line, = ax.plot([], [])
ego_line.set_color('gray')
other_line.set_color('gray')

ego_vehicle = patches.Polygon(xy = np.array([[0,0], [1,0], [1,1]]))
other_vehicle = patches.Polygon(xy = np.array([[0,0], [1,0], [1,1]]))


ego_vehicle.set_color('green')
other_vehicle.set_color('blue')

law_bound, = ax.plot([], [])
law_bound.set_color("magenta")

ego_trajectory = [[], []]
other_trajectory = [[], []]

rl_path = [[], []]
rule_path = [[], []]
rl_path_line, = ax.plot([], [])
rule_path_line, = ax.plot([], [])
rl_path_label = ax.text(0,0,'')
rl_path_label.set_rotation(-90)
rl_path_label.set_color('r')
rule_path_label = ax.text(0,0,'')
rule_path_label.set_rotation(-90)
rule_path_label.set_color('blue')

data = pd.read_csv(filename)

# words:
distance_label = ax.text(0,0,'')
distance_label.set_rotation(-90)
distance_line, = ax.plot([], [])
distance_line.set_color('k')

ego_text = ax.text(0,0,'')
ego_text.set_rotation(-90)

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
    return np.array([[x0, y0], [x1, y1], [x2, y2], [x3, y3]])

def get_lb(x ,y, yaw):
    rad = yaw*pi/180.0
    x0 = x - cos(rad)*half_w + sin(rad)*half_h
    y0 = y - sin(rad)*half_h - cos(rad)*half_w
    return x0, y0

def get_path_with_destination(current, destination, speed_direction):
    v0 = 12
    m0 = v0*speed_direction
    m1 = [0, -v0]
    t = np.linspace(0, 1, 50)
    a1 = 2*t*t*t - 3*t*t + 1
    a2 = t*t*t - 2*t*t + t
    a3 = -2*t*t*t + 3*t*t
    a4 = t*t*t - t*t 
    x = a1*current[0] + a2*m0[0] + a3*destination[0] + a4*m1[0]
    y = a1*current[1] + a2*m0[1] + a3*destination[1] + a4*m1[1]
    return x, y

frames = int(data.shape[0]/FRAME_CUT) - 150 - BEGINOMIT

def update(frame):
    global FRAME_CUT
    global BEGINOMIT
    frame = frame*FRAME_CUT + BEGINOMIT
    global start_lane_change
    global last_law
    data_line = data.iloc[frame]
    
    last_data = data.iloc[frame - 10]
    next_data = data.iloc[frame + 10]
    speed = np.array([next_data[0] - last_data[0], next_data[1] - last_data[1] - 1e-5])
    speed_direction = speed/np.linalg.norm(speed)
    # print(frame)


    ego_x = data_line[0]
    ego_y = data_line[1]
    ego_yaw = data_line[2]

    other_x = data_line[3]
    other_y = data_line[4]

    rl_action = data_line[5]
    real_action = data_line[6]

    law = data_line[7]
    if law < -0.5 and not start_lane_change:
        law_bound.set_data([], [])
    else:
        if ego_x < 207.2:
            if law < -0.5:
                law = last_law
            law_bound.set_data([other_x - 2, other_x + half_h], [other_y - law, other_y - law])
            distance_label.set_position((other_x + 1.5, other_y - law - 15))
            distance_label.set_text("law bound\n distance: {:.2f}m".format(law+0.01))
            distance_line.set_data([other_x + (half_h-2)/2, other_x + 1.5], [other_y - law, other_y - law - 3])
        else:
            law_bound.set_data([], [])
            distance_label.set_position((0,0))
            distance_label.set_text(" ")
            distance_line.set_data([], [])


    ego_trajectory[0].append(ego_x)
    ego_trajectory[1].append(ego_y)
    ego_line.set_data(ego_trajectory[0], ego_trajectory[1])

    other_trajectory[0].append(other_x)
    other_trajectory[1].append(other_y)
    other_line.set_data(other_trajectory[0], other_trajectory[1])

    # l, b = get_lb(ego_x, ego_y, ego_yaw)
    ego_vehicle.set_xy(get_bounds(ego_x, ego_y, ego_yaw))
    # ego_vehicle.angle = ego_yaw

    # l, b = get_lb(other_x, other_y, -90)
    other_vehicle.set_xy(get_bounds(other_x, other_y, -90))


    ax.add_patch(ego_vehicle)
    ax.add_patch(other_vehicle)

    # before lane change
    ego_text.set_position((ego_x - 4.0, ego_y-4))
    ego_text.set_text('RL policy')
    
    
    if ego_y - 10 > other_y + 2:
        rl_path_x, rl_path_y = get_path_with_destination([ego_x, ego_y], [ego_x, ego_y-10], speed_direction=speed_direction)
    else:
        rl_path_x, rl_path_y = get_path_with_destination([ego_x, ego_y], [203.8, ego_y-10], speed_direction=speed_direction)
    rl_path_line.set_data(rl_path_x, rl_path_y)
    rl_path_line.set_color('green')
    
    rule_path_line.set_data([],[])
    
        
    rl_path_label.set_text("")
    rl_path_label.set_position((0,0))
    
    rule_path_label.set_text("")
    rule_path_label.set_position((0,0))

    if rl_action > -0.5:
        # ego_text.set_position((0,0))
        # ego_text.set_text('')
        # start trying to back:
        if rl_action == 0:
            # rl: unsafe lane change maneuver
            ego_vehicle.set_color('green')

        
        elif real_action == 0:            
            ego_vehicle.set_color('red')
            # ego_text.set_position((ego_x - 6.0, ego_y-15))
            # ego_text.set_text('RL action: lane change\n prohibited by law monitor')
            rl_path_x, rl_path_y = get_path_with_destination([ego_x, ego_y], [207.0, ego_y-10], speed_direction=speed_direction)
            rl_path_line.set_data(rl_path_x, rl_path_y)
            rl_path_line.set_color('red')
            
            rule_path_x, rule_path_y = get_path_with_destination([ego_x, ego_y], [ego_x, ego_y-10], speed_direction=speed_direction)
            rule_path_line.set_data(rule_path_x, rule_path_y)
            rule_path_line.set_color('blue')
            
            ego_text.set_position((ego_x - 5.5, ego_y-10))
            ego_text.set_text('     RL action prohibited\n   backup policy activated')
            
            rl_path_label.set_text("rl action")
            rl_path_label.set_position((rl_path_x[-1]-1, rl_path_y[-1]-8))
            
            rule_path_label.set_text("backup action")
            rule_path_label.set_position((rule_path_x[-1]-1, rule_path_y[-1]-12))

        else:
            start_lane_change = True
            ego_vehicle.set_color('green')
            last_law = law
            
            rl_path_x, rl_path_y = get_path_with_destination([ego_x, ego_y], [207.0, ego_y-10], speed_direction=speed_direction)
            rl_path_line.set_data(rl_path_x, rl_path_y)
            rl_path_line.set_color('green')
            # ego_text.set_position((ego_x - 6.0, ego_y-5))
            # ego_text.set_text('lane changing\n obeying law')
        
        # ego_vehicle.set_color('red')
        # start_lane_change = True
        # last_law = law
        # ego_text.set_position((ego_x - 6.0, ego_y-5))
        # ego_text.set_text('lane changing\n breaking law')

    if start_lane_change:
        # ego_text.set_position((0,0))
        # ego_text.set_text('')
        if ego_x < 207.2:
            # lane change performing
            rl_path_x, rl_path_y = get_path_with_destination([ego_x, ego_y], [207.0, ego_y-10], speed_direction=speed_direction)
            rl_path_line.set_data(rl_path_x, rl_path_y)
            rl_path_line.set_color('green')
            
            # ego_vehicle.set_color('green')
            # ego_text.set_position((ego_x - 6.0, ego_y-5))
            # ego_text.set_text('lane changing\n obeying law')


            # ego_vehicle.set_color('red')
            # ego_text.set_position((ego_x - 6.0, ego_y-5))
            # ego_text.set_text('lane changing\n breaking law')
        else:
            ego_vehicle.set_color('green')
            
            rl_path_x, rl_path_y = get_path_with_destination([ego_x, ego_y], [ego_x, ego_y-10], speed_direction=speed_direction)
            rl_path_line.set_data(rl_path_x, rl_path_y)
            rl_path_line.set_color('green')
            # ego_text.set_position((ego_x - 4.0, ego_y-8))
            # ego_text.set_text('after lane change')
            


ani = animation.FuncAnimation(fig, update, frames, repeat = False, interval = 10)
ani.save(gif_name, writer="pillow",fps=20)
# pylab.show()







