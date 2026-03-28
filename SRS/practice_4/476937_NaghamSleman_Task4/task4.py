import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

# -------------------------
# Load model
# -------------------------
model_path = r"C:\Users\Nagham\Documents\mujoco_models\task4.xml"

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# joint addresses
elbow_id = model.joint('elbow').id
wrist_id = model.joint('wrist').id

elbow_qpos = model.jnt_qposadr[elbow_id]
wrist_qpos = model.jnt_qposadr[wrist_id]

elbow_qvel = model.jnt_dofadr[elbow_id]
wrist_qvel = model.jnt_dofadr[wrist_id]

# actuator IDs
act1 = model.actuator('m_elbow').id
act2 = model.actuator('m_wrist').id

# === Sine parameters (rad) ===
AMP1  = 0.3637
FREQ1 = 1.41
BIAS1 = 0.5445

AMP2  = 0.5633
FREQ2 = 2.59
BIAS2 = -0.5498


# === PD gains ===
Kp = np.array([50.0, 40.0])
Kd = np.array([0.2, 0.2])


# safety: read ctrlrange
ctrl_min = model.actuator_ctrlrange[:, 0]
ctrl_max = model.actuator_ctrlrange[:, 1]

# simulation params
dt = model.opt.timestep
sim_time = 10.0
steps = int(sim_time / dt)
t0 = 0.0

q1_start = AMP1 * np.sin(2*np.pi*FREQ1 * t0) + BIAS1
q2_start = AMP2 * np.sin(2*np.pi*FREQ2 * t0) + BIAS2 

# -------------------------------------
# SET INITIAL STATE = START OF DESIRED MOTION
# -------------------------------------
data.qpos[elbow_qpos]  = -q1_start
data.qpos[wrist_qpos]  = -q2_start
data.qvel[elbow_qvel]  = 0
data.qvel[wrist_qvel]  = 0

mujoco.mj_forward(model, data)

# logs
t_log  = []
q1_log = []
q2_log = []
q1d_log = []
q2d_log = []

start_time = time.time()

# ======================
# RUN WITH VIEWER
# ======================
with mujoco.viewer.launch_passive(model, data) as viewer:
    for step in range(steps):
        if not viewer.is_running():
            break

        t = step*dt

        # desired sine motion
        q1_des = AMP1 * np.sin(2*np.pi*FREQ1 * t) - BIAS1
        q2_des = AMP2 * np.sin(2*np.pi*FREQ2 * t) - BIAS2

        # velocities of desired motion
        dq1_des = AMP1 * 2*np.pi*FREQ1 * np.cos(2*np.pi*FREQ1 * t)
        dq2_des = AMP2 * 2*np.pi*FREQ2 * np.cos(2*np.pi*FREQ2 * t)

        # actual states
        q1  = data.qpos[elbow_qpos]
        q2  = data.qpos[wrist_qpos]
        dq1 = data.qvel[elbow_qvel]
        dq2 = data.qvel[wrist_qvel]

        # PD control
        u1 = Kp[0] * (q1_des - q1) + Kd[0] * (dq1_des - dq1)
        u2 = Kp[1] * (q2_des - q2) + Kd[1] * (dq2_des - dq2)

        # safety clipping
        u1 = float(np.clip(u1, ctrl_min[act1], ctrl_max[act1]))
        u2 = float(np.clip(u2, ctrl_min[act2], ctrl_max[act2]))

        # apply control
        data.ctrl[act1] = u1
        data.ctrl[act2] = u2

        # step simulation
        mujoco.mj_step(model, data)
        viewer.sync()

        # logging
        t_log.append(t)
        q1_log.append(q1)
        q2_log.append(q2)
        q1d_log.append(q1_des)
        q2d_log.append(q2_des)

# ======================
# PLOTS
# ======================
plt.figure()
plt.plot(t_log, q1_log, label="q1 actual")
plt.plot(t_log, q1d_log, '--', label="q1 desired")
plt.legend(); plt.grid()
plt.xlabel("time (s)")
plt.ylabel("q1 (rad)")

plt.figure()
plt.plot(t_log, q2_log, label="q2 actual")
plt.plot(t_log, q2d_log, '--', label="q2 desired")
plt.legend(); plt.grid()
plt.xlabel("time (s)")
plt.ylabel("q2 (rad)")

plt.show()