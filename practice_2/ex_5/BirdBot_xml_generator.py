from numpy import sin, cos, deg2rad
import mujoco
import matplotlib.pyplot as plt
import os

LINK_SIZE = 0.01            # [m]
FLOOR_POSITION = -0.2145    # [m]
K = 0.06                    # [N/m] tendon's stiffness

ELECTRONICS_BOX = dict(
    name = 'base',
    x = 0.05,               # [m]  
    y = 0.095,              # [m]
    z = 0.1155,             # [m]
)
LINK_1 = dict(
    name = 'link_1',
    y0 = -0.02,             # [m]
    z0 = -0.022,            # [m]
    length = 0.0805,        # [m]
    alpha = -75,            # [m]
    joint_name = 'joint_1'
)
LINK_2 = dict(
    name = 'link_2',
    y0 = LINK_1['length'] * cos(deg2rad(LINK_1['alpha'])),
    z0 = LINK_1['length'] * sin(deg2rad(LINK_1['alpha'])),
    length = 0.110,         # [m]
    alpha = -121.2,         # [m]
    joint_name = 'joint_2'
)
LINK_3 = dict(
    name = 'link_3',
    y0 = 0.05 * cos(deg2rad(LINK_1['alpha'])),
    z0 = 0.05 * sin(deg2rad(LINK_1['alpha'])),
    length = 0.110,         # [m]
    alpha = -121.2,         # [m]
    joint_name = 'joint_3'
)
LINK_4 = dict(
    name = 'link_4',
    y0 = LINK_3['length'] * cos(deg2rad(LINK_3['alpha'])),
    z0 = LINK_3['length'] * sin(deg2rad(LINK_3['alpha'])),
    length = 0.110,         # [m]
    alpha = -74.7,          # [m]
    joint_name = 'joint_4'
)
LINK_5 = dict(
    name = 'link_5',
    y0 = LINK_4['length'] * cos(deg2rad(LINK_4['alpha'])),
    z0 = LINK_4['length'] * sin(deg2rad(LINK_4['alpha'])),
    length = 0.04,          # [m]
    alpha = -55,            # [m]
    joint_name = 'joint_5'
)
LINK_6 = dict(
    name = 'link_6',
    y0 = LINK_5['length'] * cos(deg2rad(LINK_5['alpha'])),
    z0 = LINK_5['length'] * sin(deg2rad(LINK_5['alpha'])),
    length = 0.02,          # [m]
    alpha = 0,              # [m]
    joint_name = 'joint_6'
)


def link_xml(link, tab=1, j_geom_name='', joint_limit=''):
    y, z = link['length'] * cos(deg2rad(link['alpha'])), link['length'] * sin(deg2rad(link['alpha']))
    tabs = '\t'*tab
    if j_geom_name:
        j_geom_name = f'name="{j_geom_name}" '
    if joint_limit:
        joint_limit = f'limited="true" range="{joint_limit}"'
    return f"""<body name="{link['name']}" pos="0 {link['y0']} {link['z0']}">
    {tabs}<geom type="box" size="{LINK_SIZE}" fromto="0 0 0 0 {y} {z}" material="links"/>
    {tabs}<geom type="cylinder" size="{LINK_SIZE} {LINK_SIZE}" pos="0 0 0" euler="0 90 0" material="joints" />
    {tabs}<geom {j_geom_name}type="cylinder" size="{LINK_SIZE} {LINK_SIZE}" pos="0 {y} {z}" euler="0 90 0" material="joints" />
    {tabs}<joint name="{link['joint_name']}" pos="0 0 0" axis="1 0 0" {joint_limit}/>"""


XML_TEMPLATE = f"""
<mujoco>
    <option gravity="0 0 -9.8" solver="CG"/>
    <compiler balanceinertia="true" autolimits="true"/>
    <default>
        <site rgba="1 1 1 1"/>
        <tendon stiffness="{K}" width=".003" damping="0.1"/>
    </default>    
    
    <visual>
        <global azimuth="180" elevation="0"/>
    </visual>
    
    <asset>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".5 .5 .5" rgb2="1 1 1"/>
        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
        <material name="global_tendor" rgba="1 0 0 1" />
        <material name="flexor_tendon_knee" rgba="0 0 1 1" />
        <material name="extensor_tendon_1" rgba="0 0.6 0 1" />
        <material name="extensor_tendon_2" rgba="0 1 0 1" />
        <material name="disengagement_flexor_tendon" rgba="1 0.5 0 1" />
        <material name="virtual_elements" rgba="0 0 0 0.001" />
        
        <material name="links" rgba="0.75 0.75 0.75 0.3" />
        <material name="joints" rgba="0.75 0.75 0.75 0.3" />
    </asset>
    
    <worldbody>
        <geom type="plane" size="1 1 0.1" pos="0 0 {FLOOR_POSITION - 0.1}" material="grid" />
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1" />
        <body name="{ELECTRONICS_BOX['name']}" pos="0 0 0">
            <joint type="slide" axis="0 0 1"/>
            <geom type="box" size="{ELECTRONICS_BOX['x']/2} {ELECTRONICS_BOX['y']/2} {ELECTRONICS_BOX['z']/2}" rgba="0.75 0.75 0.75 0.3"/>
            <body name="knee_joint" pos="0 {LINK_1['y0']-LINK_SIZE} {-LINK_1['z0']+LINK_SIZE}">
                <joint name="joint_0" type="slide" pos="0 0 0" axis="0 0 1" range="-{LINK_SIZE} 0.1" damping="1"/>
                <geom type="capsule" size="{LINK_SIZE/3} {LINK_SIZE/3}" pos="0 0 0" euler="0 90 0" material="joints" />
                <site name="s01" pos="0 0 0"/>
            </body>
            <geom name="j0" type="cylinder" size="{LINK_SIZE} {LINK_SIZE}" pos="0 {LINK_1['y0']} {-LINK_1['z0']}" euler="0 90 0" material="joints" />
            <site name="s02" pos="0 {LINK_1['y0']-LINK_SIZE} {-LINK_1['z0']}"/>    
            <site name="s03" pos="0 {-ELECTRONICS_BOX['y']/2} 0"/>
            {link_xml(LINK_1, 3, 'j1')}
                <site name="s11" pos="0 0 0"/>
                <site name="s12" pos="0 {LINK_1['length']*cos(deg2rad(LINK_1['alpha']))+LINK_SIZE} {LINK_1['length']*sin(deg2rad(LINK_1['alpha']))}"/>
                {link_xml(LINK_2, 4, 'j2')}
                    <site name="s21" pos="0 {1/4*LINK_2['length']*cos(deg2rad(LINK_2['alpha']))+LINK_SIZE} {1/4*LINK_2['length']*sin(deg2rad(LINK_2['alpha']))}"/>
                    <site name="s22" pos="0 {2/4*LINK_2['length']*cos(deg2rad(LINK_2['alpha']))+LINK_SIZE} {2/4*LINK_2['length']*sin(deg2rad(LINK_2['alpha']))}"/>
                    <site name="s23" pos="0 {3/4*LINK_2['length']*cos(deg2rad(LINK_2['alpha']))+LINK_SIZE} {3/4*LINK_2['length']*sin(deg2rad(LINK_2['alpha']))}"/>
                    <site name="s24" pos="0 {LINK_2['length']*cos(deg2rad(LINK_2['alpha']))+LINK_SIZE} {LINK_2['length']*sin(deg2rad(LINK_2['alpha']))}"/>
                    <site name="s25" pos="0 {LINK_2['length']*cos(deg2rad(LINK_2['alpha']))-LINK_SIZE} {LINK_2['length']*sin(deg2rad(LINK_2['alpha']))}"/>
                </body>
                {link_xml(LINK_3, 4)}
                    {link_xml(LINK_4, 5, 'j4')}
                        <site name="s41" pos="0 {LINK_4['length']*cos(deg2rad(LINK_4['alpha']))-LINK_SIZE} {LINK_4['length']*sin(deg2rad(LINK_4['alpha']))}"/>
                        <site name="s42" pos="0 {LINK_4['length']*cos(deg2rad(LINK_4['alpha']))+LINK_SIZE} {LINK_4['length']*sin(deg2rad(LINK_4['alpha']))}"/>
                        <site name="s43" pos="0 {1/2*LINK_4['length']*cos(deg2rad(LINK_4['alpha']))+LINK_SIZE} {1/2*LINK_4['length']*sin(deg2rad(LINK_4['alpha']))}"/>
                        {link_xml(LINK_5, 6, 'j5', '-30 30')}
                            <site name="s51" pos="0 {LINK_5['length']*cos(deg2rad(LINK_5['alpha']))-LINK_SIZE} {LINK_5['length']*sin(deg2rad(LINK_5['alpha']))}"/>
                            <site name="s52" pos="0 {LINK_5['length']*cos(deg2rad(LINK_5['alpha']))+LINK_SIZE} {LINK_5['length']*sin(deg2rad(LINK_5['alpha']))}"/>
                            {link_xml(LINK_6, 7, joint_limit='-30 30')}
                                <site name="s61" pos="0 {2/3*LINK_6['length']*cos(deg2rad(LINK_6['alpha']))} {2/3*LINK_6['length']*sin(deg2rad(LINK_6['alpha']))}"/>
                            </body>                        
                        </body>
                    </body>
                </body>
            </body>
        </body>

    </worldbody>
    
    <equality>
        <connect name="kinematic_link" active="true" body1="link_2" body2="link_4" anchor="0 {LINK_2['length'] * cos(deg2rad(LINK_2['alpha']))} {LINK_2['length'] * sin(deg2rad(LINK_2['alpha']))}" />
    </equality>

    <actuator>
        <position name="knee_motor" joint="joint_0" kp="10"/>
        <position name="hip_motor" joint="joint_1" kp="100"/>
    </actuator>
    
    <tendon>
        <spatial name="flexor_tendon_knee" material="flexor_tendon_knee" stiffness="500">
            <site site="s01"/>
            <geom geom="j0" sidesite="s02"/>
            <site site="s21"/>
        </spatial>
    </tendon>
    <tendon>
        <spatial name="global_tendor" material="global_tendor">
            <site site="s03"/>
            <site site="s11"/>
            <site site="s12"/>
            <geom geom="j1" sidesite="s12"/>
            <site site="s25"/>
            <site site="s41"/>
            <geom geom="j4" sidesite="s41"/>
            <site site="s51"/>
            <geom geom="j5" sidesite="s51"/>
            <site site="s61"/>
        </spatial>
        <spatial name="extensor_tendon_1" material="extensor_tendon_1">
            <site site="s22"/>
            <site site="s24"/>
            <site site="s42"/>
        </spatial>
        <spatial name="extensor_tendon_2" material="extensor_tendon_2">
            <site site="s23"/>
            <site site="s24"/>
            <site site="s42"/>
            <site site="s61"/>
        </spatial>
        <spatial name="disengagement_flexor_tendon" material="disengagement_flexor_tendon">
            <site site="s43"/>
            <site site="s41"/>
            <geom geom="j4" sidesite="s41"/>
            <site site="s51"/>
            <geom geom="j5" sidesite="s51"/>
            <site site="s61"/>
        </spatial>
    </tendon>
</mujoco>
"""

dir_path = os.path.dirname(__file__)
full_path = dir_path + "/" + 'BirdBot.xml'

with open(full_path, 'w') as f:
    f.write(XML_TEMPLATE)

model = mujoco.MjModel.from_xml_string(XML_TEMPLATE)
data = mujoco.MjData(model)
mujoco.mj_step(model, data)

renderer = mujoco.Renderer(model)
renderer.update_scene(data)
plt.imshow(renderer.render())
plt.axis('off')
plt.show()
