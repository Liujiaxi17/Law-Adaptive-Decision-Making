import sys 
import glob

# import carla path
carla_version = '0.9.11'
carla_root = '/home/a/carla/CARLA_'+carla_version+'/'

try:
    sys.path.append(glob.glob(carla_root + 'PythonAPI/carla/dist/carla-'+carla_version+'-py3.7-linux-x86_64.egg' )[0])
    sys.path.append(glob.glob(carla_root + 'PythonAPI/carla/dist/carla-'+carla_version+'-py3.5-linux-x86_64.egg' )[0])
except IndexError:
    pass

import carla

import math
import numpy as np


class CubicLanePlanner():
    def __init__(self, vehicle, plan_horizon ,dt):
        '''
            dt = real timestep / total_plan_length (in s)
        '''
        self.vehicle = vehicle
        self.dt = dt / plan_horizon
        
        self.traj_length = int(1.0/self.dt)
        
    def plan(self, target_pos, target_vel):
        '''
            should in frenet coordinate
            but in this case we ignore it
            return: total traj, first p,v to track
        '''
        current_location = self.vehicle.get_location()
        current_velocity = self.vehicle.get_velocity()
        
        p0 = np.array([current_location.x, current_location.y]).reshape(1,-1)
        m0 = np.array([current_velocity.x, current_velocity.y]).reshape(1,-1)
        p1 = np.array(target_pos).reshape(1,-1)
        m1 = np.array(target_vel).reshape(1,-1)

        t = self.dt

        ts = np.linspace(0, 1, num=self.traj_length, endpoint=False).reshape(-1,1)
        ps = np.zeros((len(ts), 2))
        
        ts_2 = ts**2
        ts_3 = ts**3
        
        ps = (2*ts_3 - 3*ts_2 + 1)*p0 + (ts_3 - 2*ts_2 + ts)*m0 + (-2*ts_3 + 3*ts_2)*p1 + (ts_3 - ts_2)*m1

        # first target wp
        p = ps[1]
        v = (6*t**2 - 6*t)*p0 + (3*t**2 - 4*t + 1)*m0 + (-6*t**2 + 6*t)*p1 + (3*t**2 - 2*t)*m1
        
        return ps, p, v