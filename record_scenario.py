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

record_dir = 'records_flexible/'
acc_file = record_dir + 'acc_nolaw.txt'
nor_file = record_dir + 'normal_nolaw.txt'

from gym import spaces
import carla

import random
from planners import Planner
from controller import Controller
import time
import numpy as np

LEFT_CHANGE_DISTANCE = 10.0
# D_MIN = 9.0
D_MIN = 0.0
# D_0 = 15.0
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
        
        self.controller = Controller()

        self.acc_episode = False # flag of if the preceding car will accelerate

        self.if_check_law = if_check_law

        # self.check_point_1 = D_MIN
        # self.check_point_2 = D_0

        self.action_space = spaces.Discrete(2)

        self.state_dimension = 2
        # state = np.array([rear_d, forward_d])
        low  = np.array([0, 0])
        high = np.array([1,1])
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        self.f = None

    
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
        if self.f != None:
            self.f.close()

        # flags
        if random.random() < -10.5:
            self.acc_episode = True
            self.f = open(acc_file, 'w')
            print("acc episode")
        else:
            print("normal episode")
            self.acc_episode = False
            self.f = open(nor_file, 'w')

        self.f.write("ego_x, ego_y, ego_yaw, other_x, other_y, rl_action, final_action, distance_thre\n")
        
        self.controller.reset_vehicle()

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

        # time.sleep(5)
        ego_y = self.vehicle_ego.get_location().y
        other_y = self.vehicle_other.get_location().y
        distance = other_y - ego_y 
        self.world.tick()

        while (not self.has_changed_left or distance < 0):
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
        other_y = self.vehicle_other.get_location().y

        distance = other_y - ego_y # since y is decreasing, distance is the leading value of ego to other

        ego_vy = self.vehicle_ego.get_velocity().y
        other_vy = self.vehicle_other.get_velocity().y
        
        delta_v = other_vy - ego_vy

        if not self.is_changing_left and not self.has_changed_left:
            # not start change left
            if distance > - LEFT_CHANGE_DISTANCE:
                self.is_changing_left = self.controller.left_lane_change()
            else:
                self.controller.straight_forward()
        elif self.is_changing_left:
            self.is_changing_left = self.controller.left_lane_change()
            self.has_changed_left = not self.is_changing_left
        else:
            self.controller.straight_forward()

        if self.has_changed_left:
            self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.25, steer = 0, brake = 0))
        else:
            self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.18, steer = 0, brake = 0))
        
        ego_x = self.vehicle_ego.get_location().x
        self.spectator_transform.location.x = ego_x
        self.spectator_transform.location.y = ego_y + 15.9
        self.spectator.set_transform(self.spectator_transform)
        
        self.record_with_out_law()
        
        self.world.tick()

    def closing_steps(self):
        if self.is_changing_right:
            self.is_changing_right = self.controller.right_lane_change()
            self.has_changing_right = not self.is_changing_right
        else:
            self.controller.acc_straight_forward()

        ego_x = self.vehicle_ego.get_location().x
        ego_y = self.vehicle_ego.get_location().y
        self.spectator_transform.location.x = ego_x
        self.spectator_transform.location.y = ego_y + 15.9
        self.spectator.set_transform(self.spectator_transform)

        self.world.tick()
        self.record_with_out_law()


    def step(self, action):
        '''
            return: 
            True: continue this episode
            False: end the episode and reset the scenario
                    due to collision or finish 
        '''

        ego_y = self.vehicle_ego.get_location().y
        other_y = self.vehicle_other.get_location().y

        distance = other_y - ego_y # since y is decreasing, distance is the leading value of ego to other

        ego_vy = self.vehicle_ego.get_velocity().y
        other_vy = self.vehicle_other.get_velocity().y
        
        delta_v = other_vy - ego_vy

        change_y = 0

        if ego_y > -60 and not self.collision:
        # has changed left
            if not self.is_changing_right and not self.has_changing_right:
                if action == 1 and self.check_law(distance, delta_v):
                    self.is_changing_right = self.controller.right_lane_change()
                    self.record(action, 1, max(D_MIN - 0.01, D_0 - delta_v*TAU))
                else: 
                    self.controller.straight_forward()
                    self.record(action, 0, max(D_MIN - 0.01, D_0 - delta_v*TAU))
            else:
                ego_y = self.vehicle_ego.get_location().y
                change_y = ego_y/50
                while(ego_y > 0 and not self.collision):
                    self.closing_steps()
                    ego_y = self.vehicle_ego.get_location().y


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

        self.world.tick()

        if self.has_changed_left and self.acc_episode:
        # forward car: constant speed but with a probability to accelerate
            self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.25, steer = 0, brake = 0))
        else:
            self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.18, steer = 0, brake = 0))

        self.spectator_transform.location.x = ego_x
        self.spectator_transform.location.y = ego_y + 15.9
        self.spectator.set_transform(self.spectator_transform)


        # print("step finish")
        
        reward = change_y
        done = False

        if ego_y < 0.1:
            done = True
            if ego_x < 205.5:
                reward = -20
            print("finish task")

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
        print("distance is ", distance, " required distance is ", max(D_MIN - 0.05, D_0 - delta_v*TAU))
        
        return distance > max(D_MIN - 0.01, D_0 - delta_v*TAU)

    def stop(self):
        try:
            if self.f != None:
                self.f.close()
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

        # set spectator
        spectator = self.world.get_spectator()
        # print(spectator.get_transform())
        # raise BaseException
        transform = carla.Transform(carla.Location(x=206.667450, y=125.129417, z=39.518433), carla.Rotation(pitch=-35.576168, yaw=-85.422363, roll=0.000080))
        # for following
        spectator.set_transform(transform)
        
        # Town 01
        # self.ego_init_transform = carla.Transform(carla.Location(310,129.750,0.3),carla.Rotation(0,180,0))
        # self.other_init_transform = carla.Transform(carla.Location(295,129.750,0.3),carla.Rotation(0,180,0))

        # Town 05
        self.ego_init_transform = carla.Transform(carla.Location(207,95,0.5),carla.Rotation(0,270,0))
        self.other_init_transform = carla.Transform(carla.Location(207,80,0.5),carla.Rotation(0,270,0))

        self.spectator = spectator
        self.spectator_transform = carla.Transform(carla.Location(x=207, y=110.9, z=9.4389), carla.Rotation(pitch=-15.576168, yaw=-90, roll=0.000000))
        self.spectator.set_transform(self.spectator_transform)


        self.world.tick()
    
    def init_blueprint(self):

        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_ego_bp = random.choice(self.blueprint_library.filter('vehicle.mercedes-benz.coupe'))
        self.vehicle_other_bp = random.choice(self.blueprint_library.filter('vehicle.bmw.grandtourer'))
        self.vehicle_ego = None
        self.vehicle_other = None

        self.collision_sensor_bp = self.blueprint_library.find('sensor.other.collision')
        self.collision_sensor = None

    def record(self, rl_action, final_action, law_mark):
        '''
            record message:
                ego position
                other position
                rl decision
                final decision
                law mark
        '''

        ego_x = self.vehicle_ego.get_location().x
        ego_y = self.vehicle_ego.get_location().y
        ego_yaw = self.vehicle_ego.get_transform().rotation.yaw
        other_x = self.vehicle_other.get_location().x
        other_y = self.vehicle_other.get_location().y



        self.f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(ego_x, ego_y, ego_yaw, other_x, other_y, rl_action, final_action, law_mark))


    def record_with_out_law(self):
        '''
            record vehicles' position
        '''
        ego_x = self.vehicle_ego.get_location().x
        ego_y = self.vehicle_ego.get_location().y
        ego_yaw = self.vehicle_ego.get_transform().rotation.yaw

        other_x = self.vehicle_other.get_location().x
        other_y = self.vehicle_other.get_location().y

        self.f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(ego_x, ego_y, ego_yaw, other_x, other_y, -1, -1, -1))


if __name__ == "__main__":
    simulator = CarlaSimulation()
    
    try:
        while(True):
            simulator.reset()
            while(simulator.step(0)):
                pass
    except KeyboardInterrupt:
        simulator.stop()
    except BaseException as e:
        simulator.stop()
        raise e
        