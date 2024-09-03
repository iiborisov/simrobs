import mujoco
# import matplotlib.pyplot as plt
import mujoco_viewer
import mediapy as media

# import time
# import itertools
# import numpy as np

xml = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -30">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
      <geom name="blue_cylinder" type="cylinder" pos="0.1 -0.2 0.0" size="0.1 0.2" rgba="0 0 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_kinematics(model, data)

# regarding model
print(f"\nThe number of geoms is {model.ngeom}. Geom colors are {model.geom_rgba}")
print(model.geom("red_box"))
print(model.geom("red_box").pos)

id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'green_sphere')
id_2 = model.geom('red_box').id
id_2_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'red_box')
print(f"\nID of the red_box is {id_2}")
print(f"Name of the object with ID={id_1} is {model.geom(id_1).name}")
print(f"Name of the object with ID={id_2} is {model.geom(id_2).name}")
print(f"Pos of the object with ID={id_2} is {model.geom(id_2).pos}")
print(f"Quat of the object with ID={id_2_2} is {model.geom(id_2_2).quat}")


# regarding data
print(f"\nTime is {data.time}")
print(f"Pos of all geoms is {data.geom_xpos}")
print(f"q_pos of all geoms is {data.qpos}")
print(f"q_vel of all geoms is {data.qvel}")

# options
model.opt.gravity = [0, 0, 9.81]
print(f"Timestep is {model.opt.timestep}")
print(f"Gravity is {model.opt.gravity}")

print('Total number of DoFs in the model:', model.nv)
print('Generalized positions:', data.qpos)
print('Generalized velocities:', data.qvel)

viewer = mujoco_viewer.MujocoViewer(model, data, title="tutorial", width=1920, height=1080)
while viewer.is_alive:
    mujoco.mj_step(model, data)

    viewer.render()
    # scene_option = mujoco.MjvOption()
    # scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

viewer.close()    
