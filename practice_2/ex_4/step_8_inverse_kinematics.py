
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from matplotlib import pyplot as plt


# Forward kinematics

# xml file (assumes this is in the same folder as this file)
xml_path = 'robot_2R.xml'
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


def controller(model, data):
    # put the controller here. This function is called inside the simulation.
    pass


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
cam.elevation = -90
cam.distance = 5
cam.lookat = np.array([0.0, 0.0, 0])


# initialize the controller
init_controller(model, data)

# set the controller
# mj.set_mjcb_control(controller)

N = 500
# q0_start = 0
# q0_end = 1.57
# q1_start = 0
# q1_end = -2*3.14
# q0 = np.linspace(q0_start, q0_end, N)
# q1 = np.linspace(q1_start, q1_end, N)
theta1 = np.pi/3
theta2 = -np.pi/2

# initial angles
data.qpos[0] = theta1  # set position
data.qpos[1] = theta2  # set position

mj.mj_forward(model, data)
position_Q = data.site_xpos[0]
# print(position_Q)
r = 0.5
center = np.array([position_Q[0]-r, position_Q[1]])

phi = np.linspace(0, 2*np.pi, N)
x_ref = center[0] + r*np.cos(phi)
y_ref = center[1] + r*np.sin(phi)
x_all = []
y_all = []


# data.site_xpos[0]         #get site position
i = 0
time = 0
dt = 0.1

while not glfw.window_should_close(window):
    time_prev = time

    while (time - time_prev < 1.0/60.0):

        # Compute J
        position_Q = data.site_xpos[0]
        jacp = np.zeros((3, 2))
        mj.mj_jac(model, data, jacp, None, position_Q, 2)
        J = jacp[[0, 1], :]
        # print(J)

        # Compute inv J
        Jinv = np.linalg.inv(J)

        # Compute dX
        dX = np.array([x_ref[i]-position_Q[0], y_ref[i]-position_Q[1]])

        # Compute dq
        dq = Jinv.dot(dX)

        x_all.append(position_Q[0])
        y_all.append(position_Q[1])

        # ipdate theta1 and theta2
        theta1 += dq[0]
        theta2 += dq[1]

        data.qpos[0] = theta1
        data.qpos[1] = theta2
        mj.mj_forward(model, data)  # forward kinematics
        time += dt
        # mj.mj_step(model, data)

    i += 1

    # print(data.site_xpos[0])

    if (i >= N):
        plt.figure(1)
        plt.plot(x_all, y_all, 'bx')
        plt.plot(x_ref, y_ref, 'r-.')
        plt.ylabel("y")
        plt.xlabel("x")
        plt.gca().set_aspect('equal')
        plt.show(block=False)
        plt.pause(5)
        plt.close()
        break
    # if (time >= simend):
    #     break

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
