import mujoco as mj
import numpy as np
import os
from matplotlib import pyplot as plt
import mujoco_viewer


# xml file (assumes this is in the same folder as this file)
xml_path = 'scene.xml'
simend = 3  # simulation time
print_camera_config = 0  # set to 1 to print camera config

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def set_torque_servo(actuator_no, flag):
    if (flag == 0):
        model.actuator_gainprm[actuator_no, 0] = 0
    else:
        model.actuator_gainprm[actuator_no, 0] = 1


def pos_controller(model, data, ref, dref, i):
    # torque control
    set_torque_servo(0, 1)
    kp = [100, 300, 10]
    kd = [5, 5, 1]

    data.ctrl[0] = kp[0]*(ref[0][i]-data.joint('hip').qpos) + \
        kd[0]*(dref[0][i]-data.joint('hip').qvel)
    data.ctrl[3] = kp[1]*(ref[1][i]-data.joint('joint_knee_lever').qpos) + \
        kd[1]*(dref[1][i]-data.joint('joint_knee_lever').qvel)
    data.ctrl[6] = kp[2]*(ref[2][i]-data.joint('joint_foot_lever_up').qpos) + \
        kd[2]*(dref[2][i]-data.joint('joint_foot_lever_up').qvel)


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


# get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Example on how to set camera configuration
cam.azimuth = 90
cam.elevation = -0
cam.distance = 2.5
cam.lookat = np.array([0.0, 0.0, 0.5])

# set reference
i = 0
time = 0
N = 60*simend
time_list = np.linspace(0, simend, N)
freq = 1.45
ref = [np.pi/12 * np.sin(2*np.pi * freq * time_list) - np.deg2rad(45),
       -np.pi/6 * np.sin(2*np.pi * freq * time_list) + np.deg2rad(98.393),
       -np.pi/12 * np.sin(2*np.pi * freq * time_list) + np.deg2rad(101.8)]

# alternative reference
# angle = -70
# ref = [np.linspace(-np.deg2rad(45), -np.deg2rad(45)+np.deg2rad(angle), int(N)),
#        np.linspace(np.deg2rad(98.393),  np.deg2rad(
#            98.393)-np.deg2rad(angle), int(N)),
#        np.linspace(np.deg2rad(101.8),  np.deg2rad(101.8)-np.deg2rad(angle), int(N))]


# derivatives of angular positions
dref = [np.gradient(ref[0], time_list),
        np.gradient(ref[1], time_list),
        np.gradient(ref[2], time_list)]

xz_ee_list = [[], []]
q_list = [[], [], []]
dq_list = [[], [], []]
tau_list = [[], [], []]
q_passive = [[], []]

# set initial angles
# data.joint('hip').qpos = - np.deg2rad(45)  # set position
# data.joint('joint_knee_lever').qpos = np.deg2rad(90)  # set position
# data.joint('joint_foot_lever_up').qpos = np.deg2rad(101.8)  # set position
# mj.mj_forward(model, data)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
for _ in range(100000000):
    if viewer.is_alive:

        time_prev = data.time

        while (data.time - time_prev < 1.0/60.0):

            tip = data.site_xpos[0]

            xz_ee_list[0].append(tip[0])
            xz_ee_list[1].append(tip[2])

            q1 = float(data.joint('hip').qpos)
            q2 = float(data.joint('joint_knee_lever').qpos)
            q3 = float(data.joint('joint_foot_lever_up').qpos)

            q_knee = float(data.joint('knee').qpos)
            q_foot = float(data.joint('ankle_passive').qpos)

            dq_1 = float(data.joint('hip').qvel)
            dq_2 = float(data.joint('joint_knee_lever').qvel)
            dq_3 = float(data.joint('joint_foot_lever_up').qvel)
            # print(np.rad2deg(q2))

            t1 = float(data.sensor("act_torque_hip").data[0])
            t2 = float(data.sensor("act_torque_knee").data[0])
            t3 = float(data.sensor("act_torque_foot").data[0])
            print(t2)

            q_list[0].append(q1)
            q_list[1].append(q2)
            q_list[2].append(q3)

            q_passive[0].append(q_knee)
            q_passive[1].append(q_foot)

            dq_list[0].append(dq_1)
            dq_list[1].append(dq_2)
            dq_list[2].append(dq_3)

            tau_list[0].append(t1)
            tau_list[1].append(t2)
            tau_list[2].append(t3)

            pos_controller(model, data, ref, dref, i)

            mj.mj_step(model, data)

        i += 1
        print(i, N)
        # if (data.time >= simend):
        if (i >= N):
            # plt.figure(1)
            # plt.title("EE position")
            # plt.plot(xz_ee_list[0], xz_ee_list[1], label='sim')
            # plt.ylabel("y")
            # plt.xlabel("x")
            # plt.show(block=False)
            # plt.gca().set_xlim([-0.1, 0.1])
            # plt.grid(axis='both')
            # plt.legend()

            # plt.figure(1)
            # plt.rcParams.update({
            #     "text.usetex": True,
            #     "font.family": "monospace",
            #     "font.monospace": 'Computer Modern Typewriter'
            # })
            time_list_sim = np.linspace(0, simend, len(q_list[0]))

            plt.figure(figsize=(20, 5), dpi=80)
            ang_pos = plt.subplot(131)
            # ang_pos.rcParams.update({
            #     "text.usetex": True,
            #     "font.family": "monospace",
            #     "font.monospace": 'Computer Modern Typewriter'
            # })
            ang_pos.set_title("Angular positions")
            ang_pos.plot(time_list, np.rad2deg(ref[0]), linestyle='--',
                         color='tab:blue', label='$q_1$ ref')
            ang_pos.plot(time_list_sim, np.rad2deg(q_list[0]),
                         color='tab:blue', label='$q_1$ sim')
            ang_pos.plot(time_list, np.rad2deg(ref[1]), linestyle='--',
                         color='tab:red', label='$q_2$ ref')
            ang_pos.plot(time_list_sim, np.rad2deg(q_list[1]),
                         color='tab:red', label='$q_2$ sim')
            ang_pos.plot(time_list, np.rad2deg(ref[2]), linestyle='--',
                         color='tab:orange', label='$q_3$ ref')
            ang_pos.plot(time_list_sim, np.rad2deg(q_list[2]),
                         color='tab:orange', label='$q_3$ sim')
            ang_pos.plot(time_list_sim, np.rad2deg(q_passive[0]),
                         color='tab:purple', label='$q_{knee}$ sim')
            ang_pos.plot(time_list_sim, np.rad2deg(q_passive[1]),
                         color='tab:cyan', label='$q_{foot}$ sim')
            ang_pos.set_ylabel("$q$ [deg]")
            ang_pos.set_xlabel("$t$ [s]")
            # ang_pos.show(block=False)
            ang_pos.grid(axis='both')
            ang_pos.legend()
            ang_pos.set_xlim([0, simend])

            ang_vel = plt.subplot(132, sharex=ang_pos)
            ang_vel.set_title("Angular velocity")
            ang_vel.plot(time_list, np.rad2deg(dref[0]), linestyle='--',
                         color='tab:blue', label='$\partial q_1$ ref')
            ang_vel.plot(time_list_sim,
                         np.rad2deg(dq_list[0]), color='tab:blue', label='$\partial q_1$ sim')
            ang_vel.plot(time_list, np.rad2deg(dref[1]), linestyle='--',
                         color='tab:red', label='$\partial q_2$ ref')
            ang_vel.plot(time_list_sim,
                         np.rad2deg(dq_list[1]), color='tab:red', label='$\partial q_2$ sim')
            ang_vel.plot(time_list, dref[2], linestyle='--',
                         color='tab:orange', label='$\partial q_3$ ref')
            ang_vel.plot(
                time_list_sim, np.rad2deg(dq_list[2]), color='tab:orange', label='$\partial q_3$ sim')
            ang_vel.set_ylabel("$\partial q$ [deg/s]")
            ang_vel.set_xlabel("$t$ [s]")
            # ang_vel.show(block=False)
            ang_vel.grid(axis='both')
            ang_vel.legend()
            ang_vel.set_xlim([0, simend])

            ang_tau = plt.subplot(133, sharex=ang_vel)
            ang_tau.set_title("Torques")
            ang_tau.plot(time_list_sim, tau_list[0],
                         label='$\\tau_1$', color='tab:blue')
            ang_tau.plot(time_list_sim, tau_list[1],
                         label='$\\tau_2$', color='tab:red')
            ang_tau.plot(time_list_sim, tau_list[2],
                         label='$\\tau_3$', color='tab:orange')
            ang_tau.set_ylabel("$\\tau$ [Nm]")
            ang_tau.set_xlabel("$t$ [s]")
            # ang_tau.show(block=False)
            ang_tau.grid(axis='both')
            ang_tau.legend()
            ang_tau.set_xlim([0, simend])

            plt.pause(5)

            plt.figure(figsize=(20, 5), dpi=80)
            ang_pos_rel = plt.subplot(131)
            # ang_pos_rel.rcParams.update({
            #     "text.usetex": True,
            #     "font.family": "monospace",
            #     "font.monospace": 'Computer Modern Typewriter'
            # })
            ang_pos_rel.set_title("Angular positions")

            ang_pos_rel.plot(np.rad2deg(q_passive[0]), np.rad2deg(q_list[0]),
                             color='tab:blue', label='$q_1$ sim')

            ang_pos_rel.plot(np.rad2deg(q_passive[0]), np.rad2deg(q_list[1]),
                             color='tab:red', label='$q_2$ sim')

            ang_pos_rel.plot(np.rad2deg(q_passive[0]), np.rad2deg(q_list[2]),
                             color='tab:orange', label='$q_3$ sim')
            ang_pos_rel.plot(np.rad2deg(q_passive[0]), np.rad2deg(q_passive[0]),
                             color='tab:purple', label='$q_{knee}$ sim')
            ang_pos_rel.plot(np.rad2deg(q_passive[0]), np.rad2deg(q_passive[1]),
                             color='tab:cyan', label='$q_{foot}$ sim')
            ang_pos_rel.set_ylabel("$q$ [deg]")
            ang_pos_rel.set_xlabel("$q_{knee}$ [deg]")
            # ang_pos_rel.show(block=False)
            ang_pos_rel.grid(axis='both')
            ang_pos_rel.legend()
            ang_pos_rel.set_xlim(
                [min(np.rad2deg(q_passive[0])), max(np.rad2deg(q_passive[0]))])

            ang_vel_rel = plt.subplot(132, sharex=ang_pos)
            ang_vel_rel.set_title("Angular velocity")
            ang_vel_rel.plot(np.rad2deg(q_passive[0]),
                             np.rad2deg(dq_list[0]), color='tab:blue', label='$\partial q_1$ sim')
            ang_vel_rel.plot(np.rad2deg(q_passive[0]),
                             np.rad2deg(q_passive[0]), color='tab:red', label='$\partial q_2$ sim')
            ang_vel_rel.plot(
                np.rad2deg(q_passive[0]), np.rad2deg(dq_list[2]), color='tab:orange', label='$\partial q_3$ sim')
            ang_vel_rel.set_ylabel("$\partial q$ [deg/s]")
            ang_vel_rel.set_xlabel("$q_{knee}$ [deg]")
            # ang_vel_rel.show(block=False)
            ang_vel_rel.grid(axis='both')
            ang_vel_rel.legend()
            ang_vel_rel.set_xlim(
                [min(np.rad2deg(q_passive[0])), max(np.rad2deg(q_passive[0]))])

            ang_tau_rel = plt.subplot(133, sharex=ang_vel)
            ang_tau_rel.set_title("Torques")
            ang_tau_rel.plot(np.rad2deg(q_passive[0]), tau_list[0],
                             label='$\\tau_1$', color='tab:blue')
            ang_tau_rel.plot(np.rad2deg(q_passive[0]), tau_list[1],
                             label='$\\tau_2$', color='tab:red')
            ang_tau_rel.plot(np.rad2deg(q_passive[0]), tau_list[2],
                             label='$\\tau_3$', color='tab:orange')
            ang_tau_rel.set_ylabel("$\\tau$ [Nm]")
            ang_tau_rel.set_xlabel("$q_{knee}$ [deg]")
            # ang_tau_rel.show(block=False)
            ang_tau_rel.grid(axis='both')
            ang_tau_rel.legend()
            ang_tau_rel.set_xlim(
                [min(np.rad2deg(q_passive[0])), max(np.rad2deg(q_passive[0]))])

            plt.pause(100)
            plt.close()
            break

        # print(f"Height is {data.qpos[0]:.2f}, q1={data.qpos[1]:.2f}, q2={data.qpos[2]:.2f}")
        # print(data.site_xpos[0])
        # print(f"Knee joint position is {data.qpos[4]} ")
        # print(f"Hip joint position is {data.joint('hip').qpos} ")
        # print(f"Knee joint position is {data.joint('joint_knee_lever').qpos} ")

        # print camera configuration (help to initialize the view)
        if (print_camera_config == 1):
            print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =',
                  cam.elevation, ';', 'cam.distance = ', cam.distance)
            print('cam.lookat =np.array([', cam.lookat[0],
                  ',', cam.lookat[1], ',', cam.lookat[2], '])')

        viewer.render()
    else:
        break

# close
viewer.close()
