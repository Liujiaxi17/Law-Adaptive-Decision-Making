1. 预配置
    start_carla.bash     各py文件中的carla_root等需要按照本机carla位置改动
    carla_version = 0.9.11
    python_version = 3.7
    其他版本可能出现bug，尤其是carla版本最好不要改

2. 程序启动方法
    a) 启动carla
        bash start_carla.bash

    b) 测试
        python test_model.py

    c) 训练
        models/ model.zip是已经训练的模型
        重新训练可以
        python train_model.py

3. 文件说明：
    --main_scenario.py
        不带RL的基础场景，场景内物体行为固定
        可以python main_scenario.py直接运行

    --train/record/test_scenario.py
        用于训练/记录/测试的场景，基于main_scenario

    --test/train_model.py
        用于测试和训练模型

    --controller.py
        用于控制换道行为

    --draw_fig.py
        用于可视化记录，生成gif图像

    --get_spectator_trans.py
        获取当前时刻carla中的观察坐标

    --start_carla.bash
        启动carla

    --planners.py
        文件已经弃用