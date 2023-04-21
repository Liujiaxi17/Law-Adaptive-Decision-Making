import gym
from train_scenario import CarlaSimulation

# import numpy as np
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

# from stable_baselines.deepq.policies import MlpPolicy
from stable_baselines.deepq.policies import FeedForwardPolicy

from stable_baselines import DQN

TRING_STEPS = 50000

overtake_env = CarlaSimulation()
model_path = "models/model.zip"
model_break_path = "models/model_temp.zip"
log_path = "models/log"

try:
    model = DQN.load(model_path,env=overtake_env, tensorboard_log=log_path)
    print("load saved model")
except:
    model = DQN(FeedForwardPolicy, env=overtake_env, tensorboard_log=log_path,policy_kwargs={"layers": [64, 64, 64],    
                                                            "feature_extraction":'fullyconnected'}) #, verbose=1
    print("build new model")
try:
    for i in range(1):
        model.learn(total_timesteps=TRING_STEPS, train_cycle= i)
        model.save(model_path)

        del model # remove to demonstrate saving and loading
        model = DQN.load(model_path,env=overtake_env, tensorboard_log=log_path)
        print("train step", (1+i)*TRING_STEPS)
except BaseException as e:
    overtake_env.stop()
    model.save(model_break_path)
    raise e


overtake_env.stop()
