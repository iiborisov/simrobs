import mujoco
import mujoco_viewer
# import os

xml = """
<mujoco>
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size=".1 .2 .3" rgba="0 .9 0 1"/>
    </body>

    <composite type="grid" count="10 10 1" spacing="0.05" offset="0 .5 1">
      <skin inflate="0.001" subgrid="3" texcoord="true"/>
      <geom size=".02"/>
      <pin coord="0 0"/>
      <pin coord="8 0"/>
    </composite>

    <!-- <body pos="0 0 2">
            <freejoint/>
            <composite type="box" count="7 7 7" spacing="0.04">
                <skin texcoord="true"  rgba=".7 .7 .7 1"/>
                <geom type="capsule" size=".015 0.05" rgba=".8 .2 .1 1"/>
            </composite>
        </body>  -->
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_step(model, data)



viewer = mujoco_viewer.MujocoViewer(model, data)

for _ in range(10000):
    if viewer.is_alive:
        mujoco.mj_step(model, data)
        viewer.render()

        # print(data.energy)
    else:
        break

viewer.close()
