"""Численный разбор одного состояния: инерции, M/C/N, wrench и динамика."""
import json
import numpy as np
if __package__:
    from .model import Robot,ad
else:
    from model import Robot,ad


def main():
    robot=Robot();q=np.array([.3,.35,-.55]);dq=np.array([.2,-.3,.4]);ddq=np.array([.5,-.1,.2])
    tau=robot.inverse_dynamics(q,dq,ddq)
    T=np.array([.4,5.,.3,.1,.2,-.1]);dT=np.array([.2,.1,-.3,.4,.1,.2]);I=robot.body_inertias()[2]
    wrench_column=I@dT-ad(T).T@I@T
    report={'q':q.tolist(),'dq':dq.tolist(),'ddq':ddq.tolist(),
            'M':robot.mass(q).tolist(),'M_from_energy':robot.mass_from_energy(q).tolist(),
            'C':robot.coriolis_matrix(q,dq).tolist(),'N':robot.gravity(q).tolist(),
            'motor_torque':tau.tolist(),'forward_acceleration':robot.forward_dynamics(q,dq,tau).tolist(),
            'kinetic_J':float(.5*dq@robot.mass(q)@dq),'potential_J':float(robot.potential(q)),
            'single_body_T':T.tolist(),'single_body_dT':dT.tolist(),
            'single_body_W_row':wrench_column.tolist(),'supplied_power_W':float(wrench_column@T)}
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
