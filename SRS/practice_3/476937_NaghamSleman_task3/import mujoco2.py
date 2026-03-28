import mujoco
import mujoco.viewer

# ----------------------------------------------------
# Load the XML model
# ----------------------------------------------------
model_path = r"C:\Users\Nagham\Documents\mujoco_models\model2.xml"

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# ----------------------------------------------------
# Launch the passive viewer
# ----------------------------------------------------
viewer = mujoco.viewer.launch_passive(model, data)

# ----------------------------------------------------
# Configure the camera
# ----------------------------------------------------
viewer.cam.lookat[:] = [0.07, 0, 0]   # Look at the middle of the arm
viewer.cam.distance = 0.6             # Move camera further
viewer.cam.elevation = -10            # Slight tilt
viewer.cam.azimuth = 90               # Side view

print("Viewer is running. Press Enter to close...")

# ----------------------------------------------------
# Wait for user input and close viewer
# ----------------------------------------------------
input()
viewer.close()
