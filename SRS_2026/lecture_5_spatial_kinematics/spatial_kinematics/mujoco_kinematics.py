"""Кинематическое проигрывание 3R: задаём q и вызываем mj_forward, НЕ mj_step."""
from pathlib import Path
import argparse
import time
import numpy as np
import mujoco
if __package__:
    from .geometry import forward_kinematics,space_jacobian,point_jacobian
else:
    from geometry import forward_kinematics,space_jacobian,point_jacobian


def sample(t):
    frequency=np.array([.6,.8,1.]);amplitude=np.array([.7,.5,.6]);phase=np.array([0.,.4,-.3])
    return amplitude*np.sin(frequency*t+phase),amplitude*frequency*np.cos(frequency*t+phase)


def run(duration=10.,viewer=False,out=None):
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot3r.xml')))
    data=mujoco.MjData(model)
    site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'tip')
    rows=[]
    def update(t):
        q,dq=sample(t)
        data.qpos[:]=q;data.qvel[:]=dq;data.time=t
        mujoco.mj_forward(model,data)
        H=forward_kinematics(q)
        np.testing.assert_allclose(data.site_xpos[site],H[:3,3],atol=1e-12)
        np.testing.assert_allclose(data.site_xmat[site].reshape(3,3),H[:3,:3],atol=1e-12)
        rows.append(np.r_[t,q,dq,data.site_xpos[site],point_jacobian(q)@dq,space_jacobian(q)@dq])
    if viewer:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(model,data) as window:
            t=0.
            while window.is_running() and t<duration:
                started=time.perf_counter()
                with window.lock():update(t)
                window.sync();time.sleep(max(0.,.01-(time.perf_counter()-started)));t+=.01
    else:
        for t in np.arange(0,duration,.01):update(t)
    rows=np.asarray(rows)
    if out is not None:
        out.mkdir(parents=True,exist_ok=True)
        np.savetxt(out/'3r_kinematics.csv',rows,delimiter=',',header='t,q1,q2,q3,dq1,dq2,dq3,x,y,z,dx,dy,dz,wx,wy,wz,vx,vy,vz',comments='')
    print(f'{len(rows)} configurations: analytical PoE == MuJoCo site pose')
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',action='store_true');parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--out',type=Path,default=Path(__file__).parent/'results')
    args=parser.parse_args()
    if args.duration<=0:parser.error('duration должна быть положительной')
    run(args.duration,args.viewer,args.out)
