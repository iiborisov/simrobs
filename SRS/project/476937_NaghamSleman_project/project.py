import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt

MODEL_PATH = r"C:\Users\Nagham\Documents\mujoco_models\project.xml"

# Load model
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data  = mujoco.MjData(model)
print("Model loaded ✔️")

# ---------------------------------------------------
# Actuator IDs
# ---------------------------------------------------
def get_act(name):
    return model.actuator(name).id

wheel_l = get_act("m_wheel_left")
wheel_r = get_act("m_wheel_right")

legs = {
    "fl": ["m_hip1_fl","m_hip2_fl","m_knee_fl","m_ankle_fl"],
    "fr": ["m_hip1_fr","m_hip2_fr","m_knee_fr","m_ankle_fr"],
    "bl": ["m_hip1_bl","m_hip2_bl","m_knee_bl","m_ankle_bl"],
    "br": ["m_hip1_br","m_hip2_br","m_knee_br","m_ankle_br"],
}
for k in legs:
    legs[k] = [get_act(a) for a in legs[k]]

fold_pose = [0, -1.0, -1.0, 0.3]
MODE = "STOP"
freq = 1.0
omega = 2*np.pi*freq
hip_amp = 0.35
knee_base = -0.7
knee_amp = 0.25
ank_amp = 0.15

def smooth_set(current, target, rate=8.0):
    return current + rate*(target - current)*model.opt.timestep

def gait(t, phase):
    hip2 = hip_amp * np.sin(omega*t + phase)
    knee = knee_base + knee_amp * np.sin(omega*t + phase)
    ank  = ank_amp * np.sin(omega*t + phase)
    return [0, hip2, knee, ank]

# ---------------------------------------------------
# Control Modes
# ---------------------------------------------------
WHEEL_SPEED = 4.0   # rad/s

def wheel_forward():
    data.ctrl[wheel_l] = WHEEL_SPEED
    data.ctrl[wheel_r] = WHEEL_SPEED
    for side in legs:
        for i, act in enumerate(legs[side]):
            data.ctrl[act] = smooth_set(data.ctrl[act], fold_pose[i], rate=6.0)

def wheel_backward():
    data.ctrl[wheel_l] = -WHEEL_SPEED
    data.ctrl[wheel_r] = -WHEEL_SPEED
    for side in legs:
        for i, act in enumerate(legs[side]):
            data.ctrl[act] = smooth_set(data.ctrl[act], fold_pose[i], rate=6.0)

def wheel_rotate():
    data.ctrl[wheel_l] =  WHEEL_SPEED
    data.ctrl[wheel_r] = -WHEEL_SPEED
    for side in legs:
        for i, act in enumerate(legs[side]):
            data.ctrl[act] = smooth_set(data.ctrl[act], fold_pose[i], rate=6.0)


def leg_forward(t):
    data.ctrl[wheel_l] = 0
    data.ctrl[wheel_r] = 0
    phases = {"fl":0,"br":0,"fr":np.pi,"bl":np.pi}
    for side in legs:
        desired = gait(t, phases[side])
        for i, act in enumerate(legs[side]):
            data.ctrl[act] = smooth_set(data.ctrl[act], desired[i])

def leg_backward(t):
    data.ctrl[wheel_l] = 0
    data.ctrl[wheel_r] = 0
    phases = {"fl":np.pi,"br":np.pi,"fr":0,"bl":0}
    for side in legs:
        desired = gait(t, phases[side])
        for i, act in enumerate(legs[side]):
            data.ctrl[act] = smooth_set(data.ctrl[act], desired[i])

def stop_all():
    for i in range(model.nu):
        data.ctrl[i] = 0

# ---------------------------------------------------
# Keyboard
# ---------------------------------------------------
def key_callback(keycode):
    global MODE
    if keycode==ord('W'):
        MODE="WHEEL_FORWARD"
    elif keycode==ord('S'):
        MODE="WHEEL_BACKWARD"
    elif keycode==ord('A'):
        MODE="WHEEL_ROTATE"
    elif keycode==ord('L'):
        MODE="LEG_FORWARD"
    elif keycode==ord('K'):
        MODE="LEG_BACKWARD"
    elif keycode==ord('X'):
        MODE="STOP"

# ---------------------------------------------------
# Logging
# ---------------------------------------------------
times = []
torque_l = []
torque_r = []

# Leg joint logs (front-left leg)
hip_log   = []
knee_log  = []
ankle_log = []

CONTROL_DT = model.opt.timestep
next_time = 0.0

# ---------------------------------------------------
# Simulation loop
# ---------------------------------------------------
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    start = time.time()
    while viewer.is_running():
        now = time.time() - start
        if now < next_time:
            time.sleep(0.0001)
            continue

        t = next_time
        next_time += CONTROL_DT

        # wheel torque logs
        times.append(t)
        torque_l.append(data.actuator_force[wheel_l])
        torque_r.append(data.actuator_force[wheel_r])

        # ---- log leg joints ----
        hip_log.append(   data.qpos[model.joint('hip2_fl').qposadr] )
        knee_log.append(  data.qpos[model.joint('knee_fl').qposadr] )
        ankle_log.append( data.qpos[model.joint('ankle_fl').qposadr] )

        # Control selection
        if MODE=="WHEEL_FORWARD":
            wheel_forward()
        elif MODE=="WHEEL_BACKWARD":
            wheel_backward()
        elif MODE=="WHEEL_ROTATE":
            wheel_rotate()
        elif MODE=="LEG_FORWARD":
            leg_forward(t)
        elif MODE=="LEG_BACKWARD":
            leg_backward(t)
        else:
            stop_all()

        mujoco.mj_step(model, data)
        viewer.sync()

# ---------------------------------------------------
# Plot wheel torque
# ---------------------------------------------------
plt.figure(figsize=(10,5))
plt.plot(times, torque_l,label="Left Wheel Torque")
plt.plot(times, torque_r,label="Right Wheel Torque")
plt.xlabel("Time [s]")
plt.ylabel("Torque [N·m]")
plt.title("Wheel Torque vs Time")
plt.legend()
plt.grid(True)
plt.show()

# ---------------------------------------------------
# Plot leg joints
# ---------------------------------------------------
plt.figure(figsize=(10,5))
plt.plot(times, hip_log,   label="Hip Joint (FL)")
plt.plot(times, knee_log,  label="Knee Joint (FL)")
plt.plot(times, ankle_log, label="Ankle Joint (FL)")
plt.xlabel("Time [s]")
plt.ylabel("Joint Angle [rad]")
plt.title("Front-Left Leg Joint Angles vs Time")
plt.legend()
plt.grid(True)
plt.show()
