import mujoco
from mujoco import viewer
from numpy import sin, arcsin, deg2rad
import matplotlib.pyplot as plt
import time
import os


SIMULATION_TIME = 5            # [sec]
XML_FILE = 'BirdBot.xml'

dir_path = os.path.dirname(__file__)

model = mujoco.MjModel.from_xml_path(dir_path + "/" + XML_FILE)
data = mujoco.MjData(model)
x = []
dx = []
t = []

A = deg2rad(70)
tc_0 = 0
th0 = 1
th1 = th0 + 0.3

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    viewer.sync()
    while viewer.is_running() and time.time() - start < SIMULATION_TIME:
        step_start = time.time()

        if step_start - start > th0 and step_start - start < th1:
            data.ctrl[1] = A
        elif step_start - start > th1 and data.ctrl[1] > 0:
            if tc_0 == 0:
                tc_0 = step_start - start
            data.ctrl[1] = A - 10*(step_start - start - tc_0)
        else:
            data.ctrl[1] = 0
            
        mujoco.mj_step(model, data)

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

        viewer.sync()
        current_time = time.time()
        time_until_next_step = model.opt.timestep - (current_time - step_start)
        x.append(data.qpos.copy())
        dx.append(data.qvel.copy())
        t.append(current_time - start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
viewer.close()
print(step_start - start)
      
f, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t, x)
ax1.set_ylabel('Position')
ax1.grid()
ax2.plot(t, dx)    
ax2.set_ylabel('Velocity')
ax2.grid()
ax2.set_xlabel('Time')
plt.show()