# import carla path
carla_root = '/home/a/carla/CARLA_0.9.11/'
carla_version = '0.9.11'
import glob
# import os
import sys
try:
    sys.path.append(glob.glob(carla_root + 'PythonAPI/carla/dist/carla-'+carla_version+'-py3.7-linux-x86_64.egg' )[0])
    sys.path.append(glob.glob(carla_root + 'PythonAPI/carla')[0])
except IndexError:
    pass

import carla
# from agents.navigation.basic_agent import BasicAgent

class Controller():
    def __init__(self):
        self.reset_vehicle()

    def reset_vehicle(self):
        self.vehicle = None
        self.is_lane_changing_left = False
        self.is_lane_changing_right = False
        self.finish_left = False

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def control(self, desired_lane_index):
        if self.vehicle == None:
            print("controlled vehicle is None!!")
            return 

        start_right = False

        # check if in lane changing process        
        if self.is_lane_changing_left:
            self.left_lane_change()
        elif self.is_lane_changing_right:
            self.right_lane_change()
        else:
            # not in lane changing process
            current_lane_index = self.get_current_lane_index()
            if desired_lane_index == current_lane_index:
                self.straight_forward()
            elif desired_lane_index < current_lane_index:
                self.right_lane_change()
                start_right = True
            else:
                self.left_lane_change()
        
        return self.finish_left, start_right

    def get_current_lane_index(self):
        # return current_lane_index
        # left lane: 1, right lane: 0
        x = self.vehicle.get_location().x
        if x < 205.5:
            return 1
        else:
            return 0
        

    def left_lane_change(self):
        # if finish, return false
        self.is_lane_changing_left = True
        x = self.vehicle.get_location().x
        if x > 205.5:
            self.vehicle.apply_control(carla.VehicleControl(throttle = 0.4, steer = -0.25))
            return True
        elif x > 203.8:
            self.vehicle.apply_control(carla.VehicleControl(throttle = 0.35, steer = +0.1))
            return True
        else:
            self.is_lane_changing_left = False
            self.finish_left = True
            self.straight_forward()
            return False

    def right_lane_change(self):
        self.is_lane_changing_right = True
        x = self.vehicle.get_location().x
        if x < 205.5:
            self.vehicle.apply_control(carla.VehicleControl(throttle = 0.4, steer = +0.25))
            return True
        elif x < 207.2:
            self.vehicle.apply_control(carla.VehicleControl(throttle = 0.35, steer = -0.1))
            return True
        else:
            self.is_lane_changing_right = False
            self.straight_forward()
            return False

    def straight_forward(self):
        # to adjust the yaw
        transform_ = self.vehicle.get_transform()
        transform_.rotation.yaw = 270
        self.vehicle.set_transform(transform_)
        
        self.vehicle.apply_control(carla.VehicleControl(throttle = 0.45, steer = 0))

    def acc_straight_forward(self):
        # to adjust the yaw
        transform_ = self.vehicle.get_transform()
        transform_.rotation.yaw = 270
        self.vehicle.set_transform(transform_)
        
        self.vehicle.apply_control(carla.VehicleControl(throttle = 0.5, steer = 0))


    

