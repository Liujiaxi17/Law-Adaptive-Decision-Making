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
import carla


class Getter():
    def __init__(self):
        self.init_carla()

    def init_carla(self):
        self.client = carla.Client("localhost", 2000)

        self.world = self.client.get_world()

        # set spectator
        self.spectator = self.world.get_spectator()

    def get_it(self):
        return self.spectator.get_transform()

if __name__ == "__main__":
    Getter_ = Getter()
    print(Getter_.get_it())
