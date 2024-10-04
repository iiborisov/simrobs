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
def set_torque(mj_data, KP, KV, theta_1, theta_2):
    data.ctrl[0] = KP * (-mj_data.qpos[0] + theta_1) + KV * (0 - mj_data.qvel[0])
    data.ctrl[1] = KP * (-mj_data.qpos[1] + theta_2) + KV * (0 - mj_data.qvel[1])

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


### Referense CIRCLE movement
SIMEND = 10
TIMESTEP = 0.002
STEP_NUM = int(SIMEND/TIMESTEP)

center_circle = [0, 0]
circle_radius = 0.5
phi = np.linspace(0, 2*np.pi, STEP_NUM)

x_ref = center_circle[0] + circle_radius * np.cos(phi)
z_ref = center_circle[1] + circle_radius * np.sin(phi)


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

         # Compute Jacobian (J)
        mujoco.mj_forward(model, data)
        position_EE = data.site_xpos[0]
        jacp = np.zeros((3, 2)) # 3 is for XYZ, 2 is for theta_1 and theta_2
        mujoco.mj_jac(model, data, jacp, None, position_EE, 2)
        J = jacp[[0,2],:] # We leave only the first and third lines associated with X and Z
        print(J)

        # Compute inverse Jacobian (J_inv)
        J_inv = np.linalg.inv(J)

        # Compute dX
        dX = np.array([x_ref[i] - position_EE[0], z_ref[i] - position_EE[2]])

        # Compute dq = J_inv*dX
        dq = J_inv.dot(dX)

        # Update theta_1 and theta_2
        theta_1 += dq[0]
        theta_2 += dq[1]

        set_torque(data, 20, 5, theta_1, theta_2)

        position_EE = data.site_xpos[0]
        EE_position_x.append(position_EE[0]) # ACTUAL end-effector X position in xpos (it is a zero element in a nested list / list into list)
        EE_position_z.append(position_EE[2]) # ACTUAL z position
    

        mujoco.mj_step(model, data)

        viewer.render()
    else:
        break

viewer.close()


### Plot end-effector movement Actual and Desired
# plt.plot(EE_position_x, EE_position_z, label='Actual position')
# plt.plot(x_ref, z_ref, label='Reference position')
# plt.legend()
# plt.axis('equal')
# plt.grid()
# plt.show()
