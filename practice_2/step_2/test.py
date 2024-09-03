import mujoco as mj
import numpy as np
import os

xml_file_name = 'two_links_robot.xml'
# xml_file_name = 'leg_site.xml'

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_file_name)

model = mj.MjModel.from_xml_path(abspath) 
data = mj.MjData(model) 

data.qpos[0] = 1.10  
data.qpos[1] = -2.27  
mj.mj_forward(model, data)

tip = data.site_xpos[0]
jacp = np.zeros((3, 2))
mj.mj_jac(model, data, jacp, None, tip, 3)
print(jacp)