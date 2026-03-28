import time
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt

# Load the MuJoCo model
model_path = r"C:\Users\Nagham\Documents\mujoco_models\model2.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data_ = mujoco.MjData(model)

# Storage for recording joint motion
joint1_positions = []
joint2_positions = []

# Get the joint indices by name
elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'elbow')
wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'wrist')

elbow_index = model.jnt_qposadr[elbow_id]
wrist_index = model.jnt_qposadr[wrist_id]

# Apply initial conditions to trigger motion
data_.qpos[elbow_index] = 0.2
data_.qpos[wrist_index] = -0.1

# Run forward pass before starting the simulation
mujoco.mj_forward(model, data_)

# Launch viewer in passive mode
with mujoco.viewer.launch_passive(model, data_) as viewer:
    start = time.time()

    while viewer.is_running() and time.time() - start < 15:
        step_start = time.time()

        # Run one physics step
        mujoco.mj_step(model, data_)
        viewer.sync()

        # Record joint angles
        joint1_positions.append(data_.qpos[elbow_index])
        joint2_positions.append(data_.qpos[wrist_index])

        # Maintain stable real-time stepping
        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)

# Plot joint trajectories
plt.figure(figsize=(12,5))
plt.plot(joint1_positions, label="Joint 1 (Elbow)")
plt.plot(joint2_positions, label="Joint 2 (Wrist)")
plt.xlabel("Time Step")
plt.ylabel("Joint Position (rad)")
plt.title("Joint Positions Over Time During Passive Simulation")
plt.grid(True)
plt.legend()
plt.show()
