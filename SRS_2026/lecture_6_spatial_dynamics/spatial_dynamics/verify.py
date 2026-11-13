"""Независимая проверка инерций, сил, энергии и MuJoCo динамики."""
from pathlib import Path
import numpy as np
import mujoco
if __package__:
    from .model import Robot,ad,kinematics
    from .geometry import adjoint,exp_se3,inverse,forward_kinematics,space_jacobian,point_jacobian
    from .rigid_body import run as rigid_run
    from .simulate import run,metrics,reference,command
else:
    from model import Robot,ad,kinematics
    from geometry import adjoint,exp_se3,inverse,forward_kinematics,space_jacobian,point_jacobian
    from rigid_body import run as rigid_run
    from simulate import run,metrics,reference,command


def main():
    robot=Robot();rng=np.random.default_rng(61)
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot3r.xml')));data=mujoco.MjData(model)
    for _ in range(30):
        q=rng.uniform(-1,1,3);dq=rng.normal(size=3);ddq=rng.normal(size=3)
        data.qpos[:]=q;data.qvel[:]=dq;data.ctrl[:]=0;data.qfrc_applied[:]=0
        mujoco.mj_forward(model,data)
        M=np.zeros((3,3))
        for i in range(3):
            col=np.zeros(3);mujoco.mj_mulM(model,data,col,np.eye(3)[i]);M[:,i]=col
        np.testing.assert_allclose(robot.mass(q),M,atol=1e-12)
        np.testing.assert_allclose(robot.mass_from_energy(q),M,atol=1e-12)
        assert np.min(np.linalg.eigvalsh(M))>0
        R,p,Jw,Jp=kinematics(q);Htip=forward_kinematics(q)
        np.testing.assert_allclose(R,Htip[:3,:3],atol=1e-12)
        np.testing.assert_allclose(p,Htip[:3,3],atol=1e-12)
        np.testing.assert_allclose(Jp,point_jacobian(q),atol=1e-12)
        F=rng.normal(size=3);moment=rng.normal(size=3)
        W=np.r_[moment+np.cross(p,F),F]
        np.testing.assert_allclose(space_jacobian(q).T@W,Jw.T@moment+Jp.T@F,atol=1e-12)
        C=robot.coriolis_matrix(q,dq);G=robot.gravity(q)
        np.testing.assert_allclose(C@dq+G,data.qfrc_bias,atol=1e-12)
        step=1e-6
        dM=(robot.mass(q+step*dq)-robot.mass(q-step*dq))/(2*step)
        A=dM-2*C;np.testing.assert_allclose(A+A.T,0.,atol=1e-9)
        gradP=np.array([(robot.potential(q+step*e)-robot.potential(q-step*e))/(2*step) for e in np.eye(3)])
        np.testing.assert_allclose(gradP,G,atol=1e-8)
        np.testing.assert_allclose(data.qacc,robot.forward_dynamics(q,dq,np.zeros(3)),atol=1e-10)
        data.qacc[:]=ddq;mujoco.mj_inverse(model,data)
        np.testing.assert_allclose(data.qfrc_inverse,robot.inverse_dynamics(q,dq,ddq),atol=1e-11)
        tau_s,_,_=command('impedance',robot,q,np.zeros(3),0.)
        def U(pose):return command('impedance',robot,pose,np.zeros(3),0.)[2]
        gradU=np.array([(U(q+step*e)-U(q-step*e))/(2*step) for e in np.eye(3)])
        np.testing.assert_allclose(tau_s-G,-gradU,atol=1e-7)
        # Нулевая мощность гироскопической связи.
        T=rng.normal(size=6);Ib=robot.body_inertias()[2];P=Ib@T
        assert abs(T@(ad(T).T@P))<1e-12
        H=exp_se3(rng.normal(size=6));Ai=adjoint(inverse(H));Inew=Ai.T@Ib@Ai;Tnew=adjoint(H)@T
        np.testing.assert_allclose(Tnew@Inew@Tnew,T@Ib@T,atol=1e-12)
        tau,_,_=command('computed-torque',robot,q,dq,.8);qr,dqr,ddqr=reference(.8)
        np.testing.assert_allclose(robot.forward_dynamics(q,dq,tau),ddqr+40*(dqr-dq)+400*(qr-q),atol=1e-10)
    rows=rigid_run();assert np.max(abs(rows[:,13]-rows[0,13]))<1e-8
    assert np.max(np.linalg.norm(rows[:,14:17]-rows[0,14:17],axis=1))<1e-4
    assert np.min(rows[:,2])<0 and np.max(rows[:,2])>0
    for method in ('gravity','computed-torque','impedance'):
        rows=run(method,duration=7.,force=method=='impedance');print(method,metrics(rows))
        assert not np.any(rows[:,-1]) and np.all(np.isfinite(rows))
        if method=='gravity':assert np.max(abs(rows[:,1:4]-rows[0,1:4]))<1e-10
        if method=='computed-torque':assert metrics(rows)['rms_tip_mm']<1.
    print('OK: energy assembly, M/C/G, skew identity, Newton-Euler power, inverse/forward MuJoCo, rigid-body invariants, 3R runs')


if __name__=='__main__':main()
