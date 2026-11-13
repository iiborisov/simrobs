"""Переход координат, винтовое движение, дрейф SO(3), PoE и якобианы 3R."""
from pathlib import Path
import argparse
import json
import numpy as np
if __package__:
    from .geometry import *
else:
    from geometry import *


def experiments(out):
    out.mkdir(parents=True,exist_ok=True)
    H_i_j = transform(exp_so3([0,0,np.pi/2]), [.3,-.2,.4])
    P_i = np.array([.2,.1,0,1.])
    T_i_i_j = np.array([0.,0.,2.,.1,.2,.3])
    W_i_i = np.array([1.,2.,3.,4.,5.,6.])
    T_i_j_j = change_twist(H_i_j,T_i_i_j)
    W_i_j = change_wrench(H_i_j,W_i_i)
    report = {'P_j':(H_i_j@P_i).tolist(), 'T_i_j_j':T_i_j_j.tolist(),
              'W_i_j':W_i_j.tolist(), 'power_i':float(W_i_i@T_i_i_j),
              'power_j':float(W_i_j@T_i_j_j)}
    # Вращение вокруг смещённой оси z и перенос вдоль неё.
    r,w = np.array([.25,0,0]),np.array([0,0,1.])
    T = np.r_[w, np.cross(r,w)+.08*w]
    times = np.linspace(0,2*np.pi,401)
    H0 = transform(o=[.55,0,0])
    poses = [exp_se3(T*t)@H0 for t in times]
    positions = np.array([H[:3,3] for H in poses])
    np.savetxt(out/'screw.csv',np.c_[times,positions],delimiter=',',header='t,x,y,z',comments='')
    # Аддитивный Euler нарушает ортогональность; экспонента сохраняет структуру.
    dt=.05; w=np.array([.3,-.4,1.]); Re=np.eye(3); Rg=np.eye(3); drift=[]
    for n in range(201):
        drift.append([n*dt,np.linalg.norm(Re.T@Re-np.eye(3)),np.linalg.norm(Rg.T@Rg-np.eye(3)),np.linalg.det(Re)])
        Re=Re+dt*skew(w)@Re
        Rg=exp_so3(dt*w)@Rg
    np.savetxt(out/'rotation_drift.csv',drift,delimiter=',',header='t,euler_orthogonality,exp_orthogonality,euler_determinant',comments='')
    q=np.array([.4,.5,-.7]);dq=np.array([.2,-.3,.4])
    H=forward_kinematics(q);Js=space_jacobian(q);Jb=body_jacobian(q)
    report.update(H_4_0=H.tolist(), Js=Js.tolist(), Jb=Jb.tolist(),
                  T_4_0_0=(Js@dq).tolist(),tip_velocity=(point_jacobian(q)@dq).tolist())
    (out/'report.json').write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(12,5),layout='constrained')
    ax=fig.add_subplot(121,projection='3d');ax.plot(*positions.T,color='#7b263d');ax.plot([r[0]]*2,[r[1]]*2,[0,.6],'--',color='#477994');ax.set(xlabel='x [m]',ylabel='y [m]',zlabel='z [m]',title='Screw motion about an offset axis')
    ax=fig.add_subplot(122);d=np.asarray(drift);ax.semilogy(d[:,0],np.maximum(d[:,1],1e-16),label='Euler');ax.semilogy(d[:,0],np.maximum(d[:,2],1e-16),label='Exponential');ax.set(xlabel='t [s]',ylabel='||R.T R - I||_F',title='Rotation structure');ax.grid(alpha=.3);ax.legend()
    fig.savefig(out/'geometry.png',dpi=160);plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=Path(__file__).parent/'results')
    experiments(parser.parse_args().out)
