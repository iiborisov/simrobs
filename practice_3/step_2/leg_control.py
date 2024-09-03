import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from matplotlib import pyplot as plt
import mujoco_viewer


# xml file (assumes this is in the same folder as this file)
xml_path = 'scene.xml'
simend = 5  # simulation time
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


def set_position_servo(actuator_no, kp):
    model.actuator_gainprm[actuator_no, 0] = kp
    model.actuator_biasprm[actuator_no, 1] = -kp


def set_velocity_servo(actuator_no, kv):
    model.actuator_gainprm[actuator_no, 0] = kv
    model.actuator_biasprm[actuator_no, 2] = -kv


def pos_controller(model, data, ref, i):
    # this function is called inside the simulation.

    # position control; position/velocity servo
    set_position_servo(1, 1e3)
    set_velocity_servo(2, 1e1)
    # data.ctrl[1] = np.pi/4
    data.ctrl[1] = ref[0][i]

    set_position_servo(4, 1e3)
    set_velocity_servo(5, 1e1)
    # data.ctrl[4] = -np.pi/2
    data.ctrl[4] = ref[1][i]


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
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
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
cam.azimuth = 90
cam.elevation = -0
cam.distance = 2.5
cam.lookat = np.array([0.0, 0.0, 0.5])

# initialize the controller
init_controller(model, data)

# set reference
i = 0
time = 0
N = 60*simend
tt = np.linspace(0, simend, N)
freq = 5
ref = [np.pi/12 * np.sin(freq * tt) + np.pi/4,
       -np.pi/6 * np.sin(freq * tt) - np.pi/2]

x_ee_all = []
z_ee_all = []
theta1_all = []
theta2_all = []

# set initial angles
data.qpos[1] = 0.78  # set position
data.qpos[2] = -1.57  # set position
mj.mj_forward(model, data)


# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
for _ in range(100000000):
    if viewer.is_alive:

        time_prev = data.time

        while (data.time - time_prev < 1.0/60.0):
            tip = data.site_xpos[0]

            x_ee_all.append(tip[0])
            z_ee_all.append(tip[2])

            theta1 = data.qpos[1]
            theta2 = data.qpos[2]

            theta1_all.append(theta1)
            theta2_all.append(theta2)

            pos_controller(model, data, ref, i)

            mj.mj_step(model, data)
            time += 0.01

        i += 1
        
        if (data.time >= simend):
            plt.figure(1)
            plt.title("EE position")
            plt.plot(x_ee_all, z_ee_all, label='sim')
            plt.ylabel("y")
            plt.xlabel("x")
            plt.show(block=False)
            plt.gca().set_xlim([-0.1, 0.1])
            # plt.gca().set_aspect('equal')
            plt.legend()

            plt.figure(2)
            ttt = np.linspace(0, simend, len(theta1_all))
            plt.title("Angular positions")
            plt.plot(ttt, theta1_all, label='q1')
            plt.plot(tt, ref[0], label='ref 1')
            plt.plot(ttt, theta2_all, label='q2')
            plt.plot(tt, ref[1], label='ref 2')
            plt.ylabel("q")
            plt.xlabel("t")
            plt.show(block=False)        
            # plt.gca().set_aspect('equal')
            plt.legend()

            plt.pause(15)
            plt.close()
            break   

        print(f"Height is {data.qpos[0]:.2f}, q1={data.qpos[1]:.2f}, q2={data.qpos[2]:.2f}")
        print(data.site_xpos[0])

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

# while not glfw.window_should_close(window):

#     # get framebuffer viewport
#     viewport_width, viewport_height = glfw.get_framebuffer_size(
#         window)
#     viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

#     # Update scene and render
#     mj.mjv_updateScene(model, data, opt, None, cam,
#                        mj.mjtCatBit.mjCAT_ALL.value, scene)
#     mj.mjr_render(viewport, scene, context)

#     # swap OpenGL buffers (blocking call due to v-sync)
#     glfw.swap_buffers(window)

#     # process pending GUI events, call GLFW callbacks
#     glfw.poll_events()

# glfw.terminate()
