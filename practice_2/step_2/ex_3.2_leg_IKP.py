import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from matplotlib import pyplot as plt

""" It is a template of Pranav Bhounsule

    The playlist of his lectures on MuJoCo can be found here 
    https://youtube.com/playlist?list=PLc7bpbeTIk75dgBVd07z6_uKN1KQkwFRK&si=OplfsMvbiLXcZYtB

    Video with position control 
    https://youtu.be/SJQZKZsvRRE?si=aaB4R6KGR8E7dlKD

    Original code here 
    tiny.cc/mujoco

    Inverse kinematics problem

    What is the difference with the previous file?

    (1) Inverse kinematics is realized
    (2) Plots are given 

 """

xml_file_name = 'leg_site.xml'
simend = 20  # simulation time
print_camera_config = 0  # set to 1 to print camera config
# this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    pass


def set_torque_servo(actuator_no, flag):
    if (flag == 0):
        model.actuator_gainprm[actuator_no, 0] = 0
    else:
        model.actuator_gainprm[actuator_no, 0] = 1


def set_position_servo(actuator_no, kp):
    model.actuator_gainprm[actuator_no, 0] = kp
    model.actuator_biasprm[actuator_no, 1] = -kp


def set_velocity_servo(actuator_no, kv):
    model.actuator_gainprm[actuator_no, 0] = kv
    model.actuator_biasprm[actuator_no, 2] = -kv


def pos_controller(model, data, ref):
    # this function is called inside the simulation.

    # position control; position/velocity servo
    set_position_servo(1, 50)
    set_velocity_servo(2, 10)
    data.ctrl[1] = ref[0]

    set_position_servo(4, 50)
    set_velocity_servo(5, 10)
    data.ctrl[4] = ref[1]


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)


def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


# get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_file_name)

# MuJoCo data structures
model = mj.MjModel.from_xml_path(abspath)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
cam.azimuth = 45
cam.elevation = -35
cam.distance = 3
cam.lookat = np.array([0.0, 0.0, 0])

# initialize the controller
init_controller(model, data)

# set the reference in absolute cartesian space
i = 0
time = 0
N = 500
tt = np.linspace(0, np.pi/2, N)  # time set

z_ref = 0.1 * np.sin(20 * tt) + 0.15
x_ref = 0.1 * np.sin(20 * tt) - 0.12
# x_ref = np.linspace(-0.012, -0.012, 500)e
x_all = []
z_all = []

# initial angles
theta1 = 1.10
theta2 = -2.27
theta1_all = []
theta2_all = []

# set initial angles
data.qpos[0] = theta1  # set position
data.qpos[1] = theta2  # set position
# mj.mj_forward(model, data)


tip = data.site_xpos[0]

jacp = np.zeros((3, 2))
mj.mj_jac(model, data, jacp, None, tip, 3)
print("pam pam")



while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):

        # 1. Compute Jacobian
        tip = data.site_xpos[0]

        jacp = np.zeros((3, 2))
        mj.mj_jac(model, data, jacp, None, tip, 3)
        jac = jacp[[0, 2], :]

        # 2. Compute inverse Jacobian Jinv
        jac_inv = np.linalg.inv(jac)

        # 3. Compute dX
        dX = np.array([x_ref[i]-tip[0],
                       z_ref[i]-tip[2]])

        # 4. Compute dq
        dq = jac_inv.dot(dX)

        x_all.append(tip[0])
        z_all.append(tip[2])

        # 5. Update theta1 and theta2
        theta1 += dq[0]
        theta2 += dq[1]

        data.qpos[0] = theta1
        data.qpos[1] = theta2

        theta1_all.append(theta1)
        theta2_all.append(theta2)

        ref = [theta1, theta2]

        pos_controller(model, data, ref)
        mj.mj_step(model, data)
        time += 0.1

    i += 1

    if (i >= N):

        tt = np.linspace(0, simend, len(theta1_all))
        plt.figure(1)
        plt.title("EE position")
        plt.plot(x_all, z_all, 'x', label='sim')
        plt.plot(x_ref, z_ref, label='ref')
        plt.ylabel("y")
        plt.xlabel("x")
        plt.show(block=False)
        # plt.gca().set_aspect('equal')
        plt.legend()

        plt.figure(2)
        plt.title("Angular positions")
        plt.plot(tt, theta1_all, label='q1')
        plt.plot(tt, theta2_all, label='q2')
        plt.ylabel("q")
        plt.xlabel("t")
        plt.show(block=False)
        # plt.gca().set_aspect('equal')
        plt.legend()

        plt.pause(15)
        plt.close()
        break

    # print(i, N)
    # print(data.site_xpos[0])
    print(f"Theta_1={data.qpos[0]:.2f} and Theta_2={data.qpos[1]:.2f}")

    if (data.time >= simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # print camera configuration (help to initialize the view)
    if (print_camera_config == 1):
        print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =',
              cam.elevation, ';', 'cam.distance = ', cam.distance)
        print('cam.lookat =np.array([', cam.lookat[0],
              ',', cam.lookat[1], ',', cam.lookat[2], '])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
