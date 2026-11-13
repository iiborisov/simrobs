"""Проверки механики, производных и замкнутых законов, а не снимков графиков."""
import numpy as np
import mujoco
from pathlib import Path
if __package__:
    from .robot_math import P, mass_matrix, coriolis, gravity, jacobian, forward_kinematics, forward_dynamics
    from .controllers import Controller, joint_reference, jacobian_dot, reference
    from .simulate import run, summarize
else:
    from robot_math import P, mass_matrix, coriolis, gravity, jacobian, forward_kinematics, forward_dynamics
    from controllers import Controller, joint_reference, jacobian_dot, reference
    from simulate import run, summarize


def main():
    rng = np.random.default_rng(42)
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot.xml')))
    data = mujoco.MjData(model)
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'tip')
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link2')
    for _ in range(25):
        q, dq = rng.uniform(-1,1,2), rng.uniform(-1,1,2)
        data.qpos[:], data.qvel[:] = q, dq
        mujoco.mj_forward(model, data)
        M = np.column_stack([np.zeros(2), np.zeros(2)])
        for i in range(2):
            column = np.zeros(2)
            mujoco.mj_mulM(model,data,column,np.eye(2)[i])
            M[:,i] = column
        np.testing.assert_allclose(M, mass_matrix(q), atol=1e-12)
        np.testing.assert_allclose(data.qfrc_bias, coriolis(q,dq)+gravity(q), atol=1e-12)
        dt = 1e-6
        np.testing.assert_allclose(jacobian_dot(q,dq), (jacobian(q+dt*dq)-jacobian(q-dt*dq))/(2*dt),atol=1e-9)
        controller = Controller('computed-torque')
        tau,_ = controller.command(q,dq,1.,np.zeros(2))
        qr,vr,ar = joint_reference(*reference(1.))
        np.testing.assert_allclose(forward_dynamics(q,dq,tau), ar+70*(vr-dq)+1225*(qr-q), atol=1e-10)
        # Мощность сохраняется при преобразовании F -> J^T F.
        F = rng.normal(size=2)
        applied = np.zeros(2)
        mujoco.mj_applyFT(model,data,np.array([F[0],0.,F[1]]),np.zeros(3),data.site_xpos[tip],body,applied)
        np.testing.assert_allclose(applied, jacobian(q).T@F, atol=1e-12)
        np.testing.assert_allclose(dq@(jacobian(q).T@F), (jacobian(q)@dq)@F,atol=1e-12)
    for t in (.3,1.1,2.5,5.):
        dt=1e-5
        _,v,a=reference(t)
        np.testing.assert_allclose((reference(t+dt)[0]-reference(t-dt)[0])/(2*dt),v,atol=1e-9)
        np.testing.assert_allclose((reference(t+dt)[1]-reference(t-dt)[1])/(2*dt),a,atol=1e-9)
    coarse, fine = Controller('admittance'), Controller('admittance')
    for i in range(2500): coarse.advance(i*.002,.002,True)
    for i in range(5000): fine.advance(i*.001,.001,True)
    np.testing.assert_allclose(coarse.offset, fine.offset, atol=1e-9)
    rows = run('computed-torque',duration=7.)
    assert summarize(rows)['rms_nominal_mm'] < 1.
    for method in ('computed-torque','impedance','admittance'):
        rows=run(method,task='interaction',duration=9.,force=True)
        assert np.all(np.isfinite(rows)) and not np.any(rows[:,-1])
        # Возврат после снятия нагрузки; податливое отклонение во время толчка.
        assert np.linalg.norm(rows[-1,5:7]-rows[-1,7:9]) < 1e-4
        if method!='computed-torque': assert np.max(rows[:,5]-rows[:,7]) > .015
        print(method, summarize(rows))
    print('OK: model, Jdot, trajectory derivatives, computed torque identity, power and MuJoCo runs')


if __name__ == '__main__': main()
