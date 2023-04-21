
# from train_scenario import CarlaSimulation
from record_scenario import CarlaSimulation

# import numpy as np
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from stable_baselines.deepq.policies import MlpPolicy
from stable_baselines import DQN
import time
TEST_EPISODE_NUM = 1
time.sleep(2)

overtake_env = CarlaSimulation(False)
model_path = "models/model.zip"
log_path = "models/log"

try:
    model = DQN.load(model_path,env=overtake_env, tensorboard_log=log_path)
    print("load saved model")
except BaseException as e:
    overtake_env.stop()
    raise e
    
try:
    for i in range(TEST_EPISODE_NUM):
        state = overtake_env.reset()

        while True:
            action, _ = model.predict(state)
            state, reward, done, _ = overtake_env.step(action=action)
            if done:
                break
        
except BaseException as e:
    overtake_env.stop()
    raise e


overtake_env.stop()
