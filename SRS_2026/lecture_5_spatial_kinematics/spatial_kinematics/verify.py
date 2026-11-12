"""Проверка определений, мощности, PoE, якобианов и независимого MuJoCo XML."""
import numpy as np
import mujoco
from pathlib import Path
if __package__:
    from .geometry import *
else:
    from geometry import *


def main():
    rng=np.random.default_rng(51)
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot3r.xml')))
    data=mujoco.MjData(model);site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'tip')
    for _ in range(50):
        X=rng.normal(size=6);H=exp_se3(X);Ad=adjoint(H)
        np.testing.assert_allclose(H@inverse(H),np.eye(4),atol=1e-12)
        np.testing.assert_allclose(H[:3,:3].T@H[:3,:3],np.eye(3),atol=1e-12)
        T=rng.normal(size=6);W=rng.normal(size=6)
        np.testing.assert_allclose(hat(Ad@T),H@hat(T)@inverse(H),atol=1e-12)
        np.testing.assert_allclose(change_wrench(H,W)@change_twist(H,T),W@T,atol=1e-12)
        q=rng.uniform(-1.3,1.3,3);dq=rng.normal(size=3);step=1e-6
        H=forward_kinematics(q);dH=(forward_kinematics(q+step*dq)-forward_kinematics(q-step*dq))/(2*step)
        np.testing.assert_allclose(H,direct_chain(q),atol=1e-12)
        np.testing.assert_allclose(space_jacobian(q)@dq,vee(dH@inverse(H)),atol=1e-9)
        np.testing.assert_allclose(body_jacobian(q)@dq,vee(inverse(H)@dH),atol=1e-9)
        np.testing.assert_allclose(point_jacobian(q)@dq,dH[:3,3],atol=1e-9)
        data.qpos[:]=q;data.qvel[:]=dq;mujoco.mj_forward(model,data)
        np.testing.assert_allclose(data.site_xpos[site],H[:3,3],atol=1e-12)
        np.testing.assert_allclose(data.site_xmat[site].reshape(3,3),H[:3,:3],atol=1e-12)
        jp,jr=np.zeros((3,3)),np.zeros((3,3));mujoco.mj_jacSite(model,data,jp,jr,site)
        np.testing.assert_allclose(jp,point_jacobian(q),atol=1e-12)
        np.testing.assert_allclose(jr,space_jacobian(q)[:3],atol=1e-12)
    np.testing.assert_allclose(exp_se3([0,0,0,1,2,3]),transform(o=[1,2,3]),atol=1e-15)
    np.testing.assert_allclose(exp_so3([1e-12,0,0]),np.eye(3)+skew([1e-12,0,0]),atol=1e-20)
    for q2,q3 in [(0,0),(.5,.7),(.5,0)]:
        print('q=',[0,q2,q3], 'rank Jp=',np.linalg.matrix_rank(point_jacobian([0,q2,q3])))
    print('OK: 50 states, frame transforms, dual wrench, PoE, Js/Jb/Jp, MuJoCo position/orientation/Jacobian, pure translation, small angles')


if __name__=='__main__':main()
