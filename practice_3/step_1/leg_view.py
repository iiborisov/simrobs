import mujoco
import mujoco_viewer
import os

""" An example how to use a not default viewer "mujoco_viewer" 
    
    The viewer can be found here
    https://github.com/rohanpsingh/mujoco-python-viewer

    To install the viewer use the following command
    pip install mujoco-python-viewer

"""

# provide the full path to the files
filename = 'scene.xml'
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + '/' + filename)

# model and data are the two main entities for MuJoCo
model = mujoco.MjModel.from_xml_path(abspath)
data = mujoco.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

# simulate and render
for _ in range(100000000):
    if viewer.is_alive:
        mujoco.mj_step(model, data)
        viewer.render()
    else:
        break

# close
viewer.close()
