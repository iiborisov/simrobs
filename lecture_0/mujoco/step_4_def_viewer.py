import mujoco
from mujoco import viewer
# from numpy import sin, arcsin, deg2rad
# import matplotlib.pyplot as plt
import time
import os

SIMULATION_TIME = 100

MODEL_NAME = 'ex_15.xml'
ROOT = os.path.abspath('')
FOLDER_PATH = os.path.join(ROOT, 'practice_1/models')
MODEL_PATH = os.path.join(FOLDER_PATH, MODEL_NAME)

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()

    while viewer.is_running() and time.time() - start < SIMULATION_TIME:
        # while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data)

        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

        # renderer.update_scene(data, scene_option=scene_option)
        viewer.sync()
        # current_time = time.time()

viewer.close()
print(step_start - start)
