import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# xml file (assumes this is in the same folder as this file)
xml_path = 'ball_game.xml'
simend = 100  # simulation time
print_camera_config = 0  # set to 1 to print camera config
# this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# новый код
vel_x = 2
vel_z = 4
_overlay = {}


def add_overlay(gridpos, text1, text2):
    if gridpos not in _overlay:
        _overlay[gridpos] = ["", ""]
    _overlay[gridpos][0] += text1 + "\n"
    _overlay[gridpos][1] += text2 + "\n"


def create_overlay(model, data):
    global vel_z

    topleft = mj.mjtGridPos.mjGRID_TOPLEFT
    topright = mj.mjtGridPos.mjGRID_TOPRIGHT
    bottomleft = mj.mjtGridPos.mjGRID_BOTTOMLEFT
    bottomright = mj.mjtGridPos.mjGRID_BOTTOMRIGHT

    add_overlay(bottomleft, "Restart", "r")
    add_overlay(bottomleft, "Jumping", "Space")
    add_overlay(bottomleft, "Forward", "W")
    add_overlay(bottomleft, "Down", "S")
    add_overlay(bottomleft, "Right", "D")
    add_overlay(bottomleft, "Left", "A")
    add_overlay(bottomleft, "Time", f"{data.time:.2f}")

    add_overlay(bottomright, "Vel z", f"{vel_z:.2f}")


def init_controller(model, data):
    # initialize the controller here. This function is called once, in the beginning
    pass


def controller(model, data):
    # put the controller here. This function is called inside the simulation.
    # pass
    # Force = -c*vx*|v| i + -c*vy*|v| j + c*vz*|v| k
    vx = data.qvel[0]
    vy = data.qvel[1]
    vz = data.qvel[2]
    v = np.sqrt(vx**2+vy**2+vz**2)
    c = 0.01
    # # in joint space
    # data.qfrc_applied[0] = -c*vx*v
    # data.qfrc_applied[1] = -c*vy*v
    # data.qfrc_applied[2] = -c*vz*v
    # in cartesian space [number of a body][x y or z]
    data.xfrc_applied[1][0] = -c*vx*v
    data.xfrc_applied[1][1] = -c*vy*v
    data.xfrc_applied[1][2] = -c*vz*v


def keyboard(window, key, scancode, act, mods):
    global vel_x
    global vel_z

    # if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
    #     mj.mj_resetData(model, data)
    #     mj.mj_forward(model, data)

    if act == glfw.PRESS and key == glfw.KEY_R:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        # initial position
        data.qpos[0] = 0  # x pos
        data.qpos[1] = 0  # y pos
        data.qpos[2] = 0.1  # z pos

    if act == glfw.PRESS and key == glfw.KEY_D:
        data.qvel[0] = vel_x

    if act == glfw.PRESS and key == glfw.KEY_SPACE:
        data.qvel[2] = vel_z  # z direction

    if act == glfw.PRESS and key == glfw.KEY_A:
        data.qvel[0] = -vel_x

    if act == glfw.PRESS and key == glfw.KEY_W:
        data.qvel[1] = vel_x

    if act == glfw.PRESS and key == glfw.KEY_S:
        data.qvel[1] = -vel_x


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
cam.elevation = -45
cam.distance = 10
cam.lookat = np.array([0.0, 0.0, 0])


# initialize the controller
init_controller(model, data)


# set the controller
mj.set_mjcb_control(controller)


def add_overlay2(gridpos, text1):

    if gridpos not in _overlay2:
        _overlay[gridpos] = [""]
    _overlay[gridpos][0] += text1 + "\n"


while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    if (data.time >= simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    create_overlay(model, data)

    add_overlay(mj.mjtGridPos.mjGRID_TOPLEFT, "SBER", "LAB")

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

    # overlay items
    for gridpos, [t1, t2] in _overlay.items():

        mj.mjr_overlay(
            mj.mjtFontScale.mjFONTSCALE_150,
            gridpos,
            viewport,
            t1,
            t2,
            context)

    # clear overlay
    _overlay.clear()

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()
