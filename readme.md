It is a very early version of the overtake case in paper *Road Traffic Law Adaptive Decision-making for Self-Driving Vehicles*

It has the main framework of the proposed method, including the scenarios, law, a simple RL agent and a very simple backup policy. However, the specific action/space design, the illegal behavior identification may be different. 

The following description is mainly from my previous readme_cn doc, which was writen long ago. Thus, they may be imprecise due to some untracked changes. 

1. prerequisites:  
    carla_version = 0.9.11  
    python_version = 3.7  

    required python packages:  
    numpy==1.21.2  
    gym==0.21.0  
    matplotlib==3.5.0  
    tensorflow==1.14.0  
    pandas==1.3.4  

    This code used stable-baselines2 as the RL framework (https://github.com/Stable-Baselines-Team/stable-baselines), which is also contained in this code.   


2. run  
    0) change the carla root with your own path in file:  
    start_carla.bash  
    change the carla_root variable in the python files.  
  
    Or just add the carla egg file path to the system environment path.   

    1) start carla  
        bash start_carla.bash  

    2) train RL agent  
        python train_model.py  

    3) test_RL agent  
        python test_model.py  

    4) test with backup policy  
        in test_model.py, initialize env with param: if_check_law=True  
        python test_model.py  

3. detailed description：  
    --main_scenario.py  
        a base carla scenario for test  
        without RL  
        python main_scenario.py to test scenario  

    --train/record/test_scenario.py  
        It seems that they are same scenario code, only for different use  

    --test/train_model.py  
        test or train model  
  
    --controller.py  
        a controller for law change, the behavior is fixed  

    --draw_fig/png.py  
        for generate gif/png  

    --get_spectator_trans.py  
        get current spectator transportation in CARLA  
        mainly for tuning  

    --utils  
        Other codes used in the paper.  
        --scenario.py  
            a new scenario file

        --controller.py  
            PID controller  

        --planners.py  
            MOBIL-based lane change controller

        --cubic_line_planner.py  
            The cubic trajectory generator.

        