"""Свободное вращение: RK4 для w и экспоненциальный midpoint для R."""
from pathlib import Path
import argparse
import numpy as np
if __package__:
    from .geometry import exp_so3
else:
    from geometry import exp_so3


def rk4(y,h,f):
    a=f(y);b=f(y+h*a/2);c=f(y+h*b/2);d=f(y+h*c)
    return y+h*(a+2*b+2*c+d)/6


def run(axis=1,duration=10.,step=.002):
    J=np.array([.02,.04,.055]);w=np.array([.04,.03,.02]);w[axis]=5.
    R=np.eye(3);rows=[]
    rhs=lambda v:-np.cross(v,J*v)/J
    for i in range(round(duration/step)+1):
        rows.append(np.r_[i*step,w,R.ravel(),.5*np.sum(J*w*w),R@(J*w)])
        half=rk4(w,step/2,rhs)
        R=R@exp_so3(half*step)
        w=rk4(w,step,rhs)
    return np.asarray(rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis',choices=('min','middle','max'),default='middle');parser.add_argument('--out',type=Path,default=Path(__file__).parent/'results-rigid')
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    rows=run(('min','middle','max').index(args.axis))
    np.savetxt(args.out/'rigid.csv',rows,delimiter=',',header='t,wx,wy,wz,'+','.join(f'R{i}{j}' for i in range(3) for j in range(3))+',kinetic,Lx_world,Ly_world,Lz_world',comments='')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    for i in range(3):axs[0].plot(rows[:,0],rows[:,i+1],label=f'w{i+1}')
    axs[0].set(xlabel='t [s]',ylabel='rad/s',title=f'Rotation about {args.axis} inertia axis');axs[0].legend();axs[0].grid(alpha=.3)
    axs[1].plot(rows[:,0],rows[:,13]-rows[0,13],label='K-K0 [J]')
    axs[1].plot(rows[:,0],np.linalg.norm(rows[:,14:17]-rows[0,14:17],axis=1),label='|L_world-L0| [Nms]');axs[1].legend();axs[1].grid(alpha=.3);axs[1].set(xlabel='t [s]',title='Invariant residuals (different units)')
    fig.savefig(args.out/'rigid.png',dpi=160);plt.close(fig)
    print('max energy drift:',np.max(abs(rows[:,13]-rows[0,13])))
    print('max world angular momentum residual:',np.max(np.linalg.norm(rows[:,14:17]-rows[0,14:17],axis=1)))


if __name__=='__main__':main()
