### INVERSE KINEMATICS
### We have a robot with two links and two rotational joints
### Given: desired end-effector position  X_ref and Y_ref.
### Find: angles of Joints Q1 and Q2

import mujoco
import mujoco_viewer
import matplotlib.pyplot as plt
import numpy as np
import os


### CONTROL
def set_torque(mj_data, KP, KV, theta_1, theta_2, x_ref_diff, z_ref_diff):

    error_1 = -mj_data.qpos[0] + theta_1
    error_2 = -mj_data.qpos[1] + theta_2

    data.ctrl[0] = KP * error_1 + KV * (x_ref_diff - mj_data.qvel[0])
    data.ctrl[1] = KP * error_2 + KV * (z_ref_diff - mj_data.qvel[1])


PATH = os.path.abspath("")
FOLDER_PATH = os.path.join(PATH, "lecture_3/inverse")
model = mujoco.MjModel.from_xml_path(FOLDER_PATH + '/' + '2r_robot_PID.xml')
data = mujoco.MjData(model)


### Initial angles in RAD, initial link's length
theta_1 = 1.9 # initial angle of first joint
theta_2 = -2.42 # initial angle of second joint
data.joint('joint_1').qpos[0] = theta_1 
data.joint('joint_2').qpos[0] = theta_2 
l_1 = 1 # first link lenght
l_2 = 1 # second link lenght


###  The parametric equations of the tangent line passing through the point 
SIMEND = 10
TIMESTEP = 0.002
STEP_NUM = int(SIMEND/TIMESTEP)

a = 0.5
b = 0.25
h = 0.5 # center of ellipse by z
k = 0.5 # center of ellipse by x
phi = np.linspace(0, 2*np.pi, 5000)
psi = 1 # a parameter representing the slope of the tangent line

x_ref =k + a * np.cos(phi) + a * psi * np.sin(phi)
z_ref =h + b * np.sin(phi) - b * psi * np.cos(phi)

x_ref_diff = np.diff(x_ref)
z_ref_diff = np.diff(z_ref)


#### Empty lists for data
EE_position_x = [] # empty lists for actual position of end-effector
EE_position_z = []


### Make a viewer
viewer = mujoco_viewer.MujocoViewer(model, data,
                                    title="2R_robot",
                                    width=1920,
                                    height=1080)

for i in range(STEP_NUM):
    if viewer.is_alive:

        # Find alpha, beta, gamma
        x = x_ref[i]
        z = z_ref[i]
        x_diff = np.take(x_ref_diff, i, mode='clip') # I dont understand BUT the TAKE function is used here to avoid the error 'IndexError: index 4999 is out of bounds for axis 0 with size 4999'
        z_diff = np.take(z_ref_diff, i, mode='clip')

        alpha_numerator = x**2 + z**2 + l_1**2 - l_2**2
        alpha_denumerator = 2 * l_1 * np.sqrt(x**2 + z**2)
        alpha = np.arccos(alpha_numerator/alpha_denumerator)
        
        beta_numerator = l_1**2 + l_2**2 - x**2 - z**2
        beta_denumerator = 2 * l_1 * l_2
        beta = np.arccos(beta_numerator/beta_denumerator)
        
        gamma = np.arctan2(x, z)

        # Find theta_1 and theta_2
        theta_1 = - gamma + alpha
        theta_2 = beta - np.pi

        set_torque(data, 20, 5, theta_1, theta_2, x_diff, z_diff)

        position_EE = data.site_xpos[0]
        EE_position_x.append(position_EE[0]) # ACTUAL end-effector X position in xpos (it is a zero element in a nested list / list into list)
        EE_position_z.append(position_EE[2]) # ACTUAL z position
            
        mujoco.mj_step(model, data)

        viewer.render()
    else:
        break

    
viewer.close()


### Plot end-effector movement Actual and Desired
plt.plot(EE_position_x, EE_position_z, label='Actual trajectory', linewidth=4)
plt.plot(x_ref, z_ref, '--', label='Reference trajectory')
plt.legend(loc='upper left')
plt.xlabel('X-Axis')
plt.ylabel('Z-Axis')
plt.axis('equal')
plt.grid()
plt.show()
