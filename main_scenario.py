# import carla path
carla_root = '/home/a/carla/CARLA_0.9.11/'
carla_version = '0.9.11'
import glob
# import os
import sys
try:
	sys.path.append(glob.glob(carla_root + 'PythonAPI/carla/dist/carla-'+carla_version+'-py3.7-linux-x86_64.egg' )[0])
except IndexError:
	pass

import carla

import random
from planners import Planner
from controller import Controller
import time

LEFT_CHANGE_DISTANCE = 10
D_MIN = 8
D_0 = 15
TAU = 3
# d_thre = max(d_min, d_0 - (v_e-v_other)*tau)

class CarlaSimulation:
    def __init__(self, if_check_law = False):

        self.init_carla()
        self.init_blueprint()
        self.init_flags()

        self.if_check_law = if_check_law
        
        self.controller = Controller()

        self.acc_episode = False # flag of if the preceding car will accelerate

        self.check_point_1 = D_MIN
        self.check_point_2 = D_0
    
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
        # flags
        self.init_flags()
        if random.random() < 0.5:
            self.acc_episode = True
            print("acc episode")
        else:
            self.acc_episode = False
        
        self.controller.reset_vehicle()

        if self.collision_sensor is not None:
            self.collision_sensor.destroy()
        if self.vehicle_ego is not None:
            self.vehicle_ego.destroy()
        if self.vehicle_other is not None:
            self.vehicle_other.destroy()

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

        self.world.tick()

    def step(self):
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
            # has changed left
            if not self.is_changing_right and not self.has_changing_right:
                if distance > self.check_point_1 and not self.has_pass_check_point_1:
                    action = 1
                    # action = self.model.predict(state)
                    if action and self.check_law(distance = distance, delta_v = delta_v):
                        self.is_changing_right = self.controller.right_lane_change()
                        print("lane change at point 1")
                    else:
                        self.controller.straight_forward()
                        print("lane change failed at point 1")
                    self.has_pass_check_point_1 = True
                elif distance > self.check_point_2 and not self.has_pass_check_point_2:
                    action = 1
                    # action = self.model.predict(state)
                    if action and self.check_law(distance = distance, delta_v = delta_v):
                        self.is_changing_right = self.controller.right_lane_change()
                        print("lane change at point 2")
                    else:
                        self.controller.straight_forward()
                        print("lane change failed at point 2")
                    self.has_pass_check_point_2 = True
                else: 
                    self.controller.straight_forward()
            elif self.is_changing_right:
                self.is_changing_right = self.controller.right_lane_change()
                self.has_changing_right = not self.is_changing_right
            else:
                self.controller.acc_straight_forward()


        ego_x = self.vehicle_ego.get_location().x
        ego_y = self.vehicle_ego.get_location().y
        ego_vx = self.vehicle_ego.get_velocity().x
        ego_vy = self.vehicle_ego.get_velocity().y

        other_y = self.vehicle_other.get_location().y
        other_vy = self.vehicle_other.get_velocity().y
        diff_d = 3*(ego_vy - other_vy)
        real_d = ego_y - other_y

        state = [ego_x, ego_y, ego_vx, ego_vy, other_y]


        self.world.tick()

        if self.has_changed_left and self.acc_episode:
        # forward car: constant speed but with a probability to accelerate
            self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.3, steer = 0, brake = 0))
        else:
            self.vehicle_other.apply_control(carla.VehicleControl(throttle = 0.18, steer = 0, brake = 0))


        # print("step finish")

        if ego_y < -80:
            return False

        if self.collision:
            # return True
            return False

        return True

    def check_law(self, distance, delta_v):
        # check if the lane change obeys the law
        # if obey, return true
        print("distance is ", distance, " required distance is ", max(D_MIN - 0.05, D_0 - delta_v*TAU))

        # spectator = self.world.get_spectator()
        # print(spectator.get_transform())
        
        if not self.if_check_law:
            return True

        return distance > max(D_MIN - 0.05, D_0 - delta_v*TAU)

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
        # settings.synchronous_mode = True
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

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
        self.ego_init_transform = carla.Transform(carla.Location(207,95,4.8),carla.Rotation(0,270,0))
        self.other_init_transform = carla.Transform(carla.Location(207,80,4.8),carla.Rotation(0,270,0))

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
    simulator = CarlaSimulation()
    
    try:
        while(True):
            simulator.reset()
            while(simulator.step()):
                pass
    except KeyboardInterrupt:
        simulator.stop()
    except BaseException as e:
        simulator.stop()
        raise e
        