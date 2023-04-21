# import carla path
carla_version = '0.9.11'
carla_root = '/home/a/carla/CARLA_'+carla_version+'/'
import glob
# import os
import sys
try:
	sys.path.append(glob.glob(carla_root + 'PythonAPI/carla/dist/carla-'+carla_version+'-py3.7-linux-x86_64.egg' )[0])
    
	sys.path.append(glob.glob(carla_root + 'PythonAPI/carla/dist/carla-'+carla_version+'-py3.5-linux-x86_64.egg' )[0])
except IndexError:
	pass


from gym import spaces
import carla

import random
from planners import BackupPlanner
from utils.controller import VehiclePIDController, DEFAULT_ARGS_LAT_CITY_DICT, DEFAULT_ARGS_LONG_CITY_DICT
import time
import numpy as np

LEFT_CHANGE_DISTANCE = 10.0
D_MIN = 9.0
D_0 = 15.0
TAU = 3.0
# d_thre = max(d_min, d_0 - (v_e-v_other)*tau)

def scale_delta_v (delta_v):
    min_v = 1.2
    max_v = 2.6
    return (delta_v - min_v) / (max_v - min_v)

def scale_d (distance):
    min_d = -5.0
    max_d = 20.0
    return (distance - min_d) / (max_d - min_d)

class CarlaSimulation:
    def __init__(self, if_check_law = False):

        self.init_carla()
        self.init_blueprint()
        self.init_flags()
        
        self.controller = VehiclePIDController(None, DEFAULT_ARGS_LAT_CITY_DICT, DEFAULT_ARGS_LONG_CITY_DICT)
        self.backplanner = BackupPlanner()

        self.lag_throttle = 0

        self.if_check_law = if_check_law

        # self.check_point_1 = D_MIN
        # self.check_point_2 = D_0

        self.action_space = spaces.Discrete(2)

        self.state_dimension = 2
        # state = np.array([rear_d, forward_d])
        low  = np.array([0, 0])
        high = np.array([1,1])
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

    def d_thre(self, delta_v):
        # return max(D_MIN - 0.01, D_0 - delta_v*TAU)
        return D_MIN - 0.01
        return D_0 - delta_v*TAU

    
    def init_flags(self):
        # flags
        self.collision = False

        self.is_changing_left = False
        self.has_changed_left = False

        self.is_changing_right = False
        self.has_changing_right = False # overtake finished

        self.has_pass_check_point_1 = False
        self.has_pass_check_point_2 = False

    def reset(self):

        self.lag_throttle = 0.18 + (0.26-0.18) * random.random()


        self.controller.reset()

        if self.collision_sensor is not None:
            self.collision_sensor.destroy()
        if self.vehicle_ego is not None:
            self.vehicle_ego.destroy()
        if self.vehicle_other is not None:
            self.vehicle_other.destroy()
        self.init_flags()


        # create new actors
        self.vehicle_ego = self.world.spawn_actor(self.vehicle_ego_bp, self.ego_init_transform)
        self.vehicle_other = self.world.spawn_actor(self.vehicle_other_bp, self.other_init_transform)
        # create sensor, with its attachment and callback function
        self.collision_sensor = self.world.spawn_actor(self.collision_sensor_bp, carla.Transform(
                                                    carla.Location(0,0,0),
                                                    carla.Rotation(0,0,0)), attach_to = self.vehicle_ego)
        self.collision_sensor.listen(self.callback)

        self.controller.set_vehicle(self.vehicle_ego)

        self.world.tick()

        # time.sleep(5)
        ego_y = self.vehicle_ego.get_location().y
        other_y = self.vehicle_other.get_location().y
        distance = other_y - ego_y 

        while (distance < 0): # ego vehicle is behind the other in the longitudinal direction
            ego_y = self.vehicle_ego.get_location().y
            other_y = self.vehicle_other.get_location().y
            distance = other_y - ego_y 

            self.initial_steps()

        self.world.tick()
        
        ego_y = self.vehicle_ego.get_location().y
        other_y = self.vehicle_other.get_location().y
        ego_vy = self.vehicle_ego.get_velocity().y
        other_vy = self.vehicle_other.get_velocity().y
        
        delta_v = other_vy - ego_vy
        distance = other_y - ego_y 


        state = np.array([scale_d (distance), scale_delta_v (delta_v)])

        return state

    def initial_steps(self):

        ego_y = self.vehicle_ego.get_location().y
        ego_x = self.vehicle_ego.get_location().x
        other_y = self.vehicle_other.get_location().y

        distance = other_y - ego_y # since y is decreasing, distance is the leading value of ego to other

        ego_vy = self.vehicle_ego.get_velocity().y
        other_vy = self.vehicle_other.get_velocity().y
        
        delta_v = other_vy - ego_vy
        
        target_location = self.vehicle_ego.get_location()
        target_speed = 2.6

        if distance > - LEFT_CHANGE_DISTANCE:
            target_location.x = 204
            target_location.y = ego_y-10
            target_waypoint = self.map.get_waypoint(target_location)
            veh_control = self.controller.run_step(target_speed=target_speed, waypoint=target_waypoint)
        else:
            target_location.y = ego_y-10
            target_waypoint = self.map.get_waypoint(target_location)
            veh_control = self.controller.run_step(target_speed=target_speed, waypoint=target_waypoint)

        self.vehicle_ego.apply_control(veh_control)
        
        self.vehicle_other.apply_control(carla.VehicleControl(throttle = self.lag_throttle, steer = 0, brake = 0))
        

        self.world.tick()


    def step(self, action):
        '''
        '''

        ego_y = self.vehicle_ego.get_location().y
        other_y = self.vehicle_other.get_location().y

        distance = other_y - ego_y # since y is decreasing, distance is the leading value of ego to other

        ego_vy = self.vehicle_ego.get_velocity().y
        other_vy = self.vehicle_other.get_velocity().y
        
        delta_v = other_vy - ego_vy

        change_y = 0
        
        target_location = self.vehicle_ego.get_location()
        target_speed = 2.8
        
        if distance > 5:
            target_location.x = 207
            target_location.y = ego_y-10
            target_waypoint = self.map.get_waypoint(target_location)
            veh_control = self.controller.run_step(target_speed=target_speed, waypoint=target_waypoint)
        else:
            target_location.y = ego_y-10
            target_waypoint = self.map.get_waypoint(target_location)
            veh_control = self.controller.run_step(target_speed=target_speed, waypoint=target_waypoint)
            
        self.vehicle_ego.apply_control(veh_control)
        

        ego_x = self.vehicle_ego.get_location().x
        ego_y = self.vehicle_ego.get_location().y
        ego_vx = self.vehicle_ego.get_velocity().x
        ego_vy = self.vehicle_ego.get_velocity().y

        other_y = self.vehicle_other.get_location().y
        other_vy = self.vehicle_other.get_velocity().y
        diff_d = 3*(ego_vy - other_vy)
        real_d = ego_y - other_y

        state = np.array([scale_d (distance), scale_delta_v (delta_v)])
        # print(state)

        # if scale_d(distance) < -1 or scale_d(distance) > 1 or scale_delta_v (delta_v) < -1 or scale_delta_v (delta_v) > 1:
        #     print(distance, delta_v)


        self.world.tick()

        # if self.has_changed_left and self.acc_episode:
        # # forward car: constant speed but with a probability to accelerate
        #     self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.25, steer = 0, brake = 0))
        # else:
        #     self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.18, steer = 0, brake = 0))

        self.vehicle_other.apply_control(carla.VehicleControl(throttle = self.lag_throttle, steer = 0, brake = 0))

        # print("step finish")
        
        reward = change_y
        done = False

        if ego_y < 0.1:
            done = True
            if ego_x < 205.5:
                reward = -20
            print("finish task with throttle", self.lag_throttle)

        if self.collision:
            # return True
            done = True
            reward = -100

        return state, reward, done, {"is_success": not self.collision}

    def check_law(self, distance, delta_v):
        # check if the lane change obeys the law
        # if obey, return true

        if not self.if_check_law:
            return True
        # spectator = self.world.get_spectator()
        # print(spectator.get_transform())
        # print("distance is ", distance, " required distance is ", max(D_MIN - 0.05, D_0 - delta_v*TAU))
        
        return distance > self.d_thre(delta_v)

    def stop(self):
        try:
            if self.collision_sensor is not None:
                self.collision_sensor.destroy()
                self.collision_sensor = None
            if self.vehicle_ego is not None:
                self.vehicle_ego.destroy()
                self.vehicle_ego = None
            if self.vehicle_other is not None:
                self.vehicle_other.destroy()
                self.vehicle_other = None
        except:
            pass
        self.world.tick()

        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

    def callback(self, event):
        if not self.collision:
            vehicle = event.other_actor
            print('Collision with %s' % vehicle.type_id)
            self.collision = True

    def init_carla(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(10.0)
        self.client.load_world('Town05')

        self.world = self.client.get_world()
        self.world.set_weather(carla.WeatherParameters(cloudiness=0, precipitation=30.0, sun_altitude_angle=70.0))
        settings = self.world.get_settings()
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        # settings.synchronous_mode = False
        self.world.apply_settings(settings)
        
        self.map = self.world.get_map()

        # set spectator
        spectator = self.world.get_spectator()
        # print(spectator.get_transform())
        # raise BaseException
        transform = carla.Transform(carla.Location(x=206.667450, y=125.129417, z=39.518433), carla.Rotation(pitch=-35.576168, yaw=-85.422363, roll=0.000080))
        spectator.set_transform(transform)
        
        # Town 01
        # self.ego_init_transform = carla.Transform(carla.Location(310,129.750,0.3),carla.Rotation(0,180,0))
        # self.other_init_transform = carla.Transform(carla.Location(295,129.750,0.3),carla.Rotation(0,180,0))

        # Town 05
        self.ego_init_transform = carla.Transform(carla.Location(207,95,0.5),carla.Rotation(0,270,0))
        self.other_init_transform = carla.Transform(carla.Location(207,83,0.5),carla.Rotation(0,270,0))

        self.world.tick()
    
    def init_blueprint(self):

        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_ego_bp = random.choice(self.blueprint_library.filter('vehicle.mercedes-benz.coupe'))
        self.vehicle_other_bp = random.choice(self.blueprint_library.filter('vehicle.bmw.grandtourer'))
        self.vehicle_ego = None
        self.vehicle_other = None

        self.collision_sensor_bp = self.blueprint_library.find('sensor.other.collision')
        self.collision_sensor = None


if __name__ == "__main__":
    simulator = CarlaSimulation(True)
    
    try:
        while(True):
            simulator.reset()
            done = False
            while(not done):
                _, _, done, _ = simulator.step(1)
    except KeyboardInterrupt:
        simulator.stop()
    except BaseException as e:
        simulator.stop()
        raise e
        