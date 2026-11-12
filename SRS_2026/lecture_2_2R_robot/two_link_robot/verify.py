"""Проверка аналитических формул по MuJoCo и проверка слежения без GUI."""
from pathlib import Path
import mujoco
import numpy as np

if __package__:
    from .robot_math import (P, forward_kinematics, inverse_kinematics, jacobian,
                             mass_matrix, coriolis, gravity, inverse_dynamics,
                             forward_dynamics, reference)
    from .simulate import run
else:
    from robot_math import (P, forward_kinematics, inverse_kinematics, jacobian,
                            mass_matrix, coriolis, gravity, inverse_dynamics,
                            forward_dynamics, reference)
    from simulate import run


def main():
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot.xml')))
    data=mujoco.MjData(model)
    rng=np.random.default_rng(42)
    assert (model.nq,model.nv,model.nu)==(2,2,2)
    errors={key:0. for key in ['position','jacobian','mass','bias','forward','inverse']}
    for _ in range(30):
        q=rng.uniform(-2,2,2); dq=rng.uniform(-1,1,2); tau=rng.uniform(-5,5,2)
        data.qpos[:]=q;data.qvel[:]=dq;data.ctrl[:]=tau
        mujoco.mj_forward(model,data)
        actual=data.site('tip').xpos[[0,2]]-np.array([0,P.base_height])
        jp=np.zeros((3,2));jr=np.zeros((3,2))
        mujoco.mj_jacSite(model,data,jp,jr,model.site('tip').id)
        # mj_mulM доступна и в старых версиях API, в отличие от новой сигнатуры mj_fullM.
        columns=[]
        for e in np.eye(2):
            result=np.zeros(2);mujoco.mj_mulM(model,data,result,e)
            columns.append(result)
        matrix=np.column_stack(columns)
        candidates={
            'position':actual-forward_kinematics(q),
            'jacobian':jp[[0,2]]-jacobian(q),
            'mass':matrix-mass_matrix(q),
            'bias':data.qfrc_bias-coriolis(q,dq)-gravity(q),
            'forward':data.qacc-forward_dynamics(q,dq,tau),
        }
        ddq=rng.uniform(-2,2,2)
        data.qacc[:]=ddq
        mujoco.mj_inverse(model,data)
        candidates['inverse']=data.qfrc_inverse-inverse_dynamics(q,dq,ddq)
        for name,delta in candidates.items():
            errors[name]=max(errors[name],float(np.max(np.abs(delta))))
    assert max(errors.values())<1e-9,errors
    for t in np.linspace(0,14,1001):
        pos,vel,q,dq=reference(t)
        assert np.linalg.norm(forward_kinematics(q)-pos)<1e-12
        assert np.linalg.norm(jacobian(q)@dq-vel)<1e-12
        assert abs(np.linalg.det(jacobian(q)))>.1
        assert np.linalg.norm(forward_kinematics(inverse_kinematics(pos,-1))-pos)<1e-12
    # Сравнение целевых скоростей с производной непрерывной ветви IK.
    for t in [0.5,1.,2.,4.,7.]:
        dt=1e-5
        numerical=(reference(t+dt)[2]-reference(t-dt)[2])/(2*dt)
        assert np.max(np.abs(numerical-reference(t)[3]))<1e-7
    print('MuJoCo',mujoco.__version__,'; максимальные расхождения:',errors)
    rms={}
    for controller in ['pd','pd-gravity']:
        rows=run(controller)
        rms[controller]=float(np.sqrt(np.mean(rows[:,15]**2)))
        assert np.all(rows[:,16]==0),'Неожиданное насыщение в эталонном сценарии'
    assert rms['pd-gravity']<.005,rms
    assert rms['pd-gravity']<rms['pd']/10,rms
    print('IK, якобиан, M, c+G, прямая/обратная динамика: OK')
    print('RMS слежения, мм:',{name:round(value*1000,3) for name,value in rms.items()})


if __name__=='__main__':
    main()
