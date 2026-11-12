"""Динамические эксперименты 3R: MuJoCo интегрирует q,qdot; регулятор задаёт момент."""
from pathlib import Path
import argparse,json,time
import numpy as np
import mujoco
if __package__:
    from .model import Robot,kinematics
    from .geometry import unskew
else:
    from model import Robot,kinematics
    from geometry import unskew

METHODS=('free','gravity','computed-torque','impedance')
COLUMNS='t q1 q2 q3 dq1 dq2 dq3 x y z xd yd zd tau1 tau2 tau3 Fx Fy Fz energy spring_energy power dissipated_power saturated'.split()


def reference(t):
    center=np.array([.3,.35,-.55]);amplitude=np.array([.2,.2,.25]);w=np.array([.8,.7,.9])
    return center+amplitude*(1-np.cos(w*t)),amplitude*w*np.sin(w*t),amplitude*w*w*np.cos(w*t)


def external_force(t,enabled):return np.array([3*np.sin(np.pi*(t-3)/2)**2 if enabled and 3<t<5 else 0.,0.,0.])


def command(method,nominal,q,dq,t):
    qr,dqr,ddqr=reference(t)
    Rd,pd,_,_=kinematics(reference(0)[0])
    if method=='free':return np.zeros(3),pd,0.
    if method=='gravity':return nominal.gravity(q),pd,0.
    if method=='computed-torque':
        target=ddqr+40*(dqr-dq)+400*(qr-q)
        return nominal.inverse_dynamics(q,dq,target),kinematics(qr)[1],0.
    R,p,Jw,Jp=kinematics(q)
    # Потенциал пространственной пружины: kp/2*||p-pd||² + ko/2*tr(I-Rd^T R).
    kp,dp,ko,do=120.,18.,2.,.8
    eR=.5*unskew(Rd.T@R-R.T@Rd)
    force=-kp*(p-pd)-dp*(Jp@dq)
    tip_moment=-ko*(R@eR)-do*(Jw@dq)
    tau=nominal.gravity(q)+Jp.T@force+Jw.T@tip_moment
    U=.5*kp*np.sum((p-pd)**2)+.5*ko*np.trace(np.eye(3)-Rd.T@R)
    return tau,pd,U


def run(method='computed-torque',duration=8.,force=False,viewer=False,step=.002,mass_scale=1.):
    model=mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot3r.xml')))
    model.opt.timestep=step;data=mujoco.MjData(model)
    nominal=Robot(mass_scale);actual=Robot();data.qpos[:]=reference(0)[0]
    site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'tip')
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'link3');rows=[]
    def tick():
        mujoco.mj_forward(model,data);t=data.time;q=data.qpos.copy();dq=data.qvel.copy()
        tau,pd,U=command(method,nominal,q,dq,t);applied=np.clip(tau,-20,20);F=external_force(t,force)
        data.ctrl[:]=applied;data.qfrc_applied[:]=0
        mujoco.mj_applyFT(model,data,F,np.zeros(3),data.site_xpos[site],body,data.qfrc_applied)
        _,p,_,Jp=kinematics(q)
        power=applied@dq+F@(Jp@dq)
        rows.append(np.r_[t,q,dq,p,pd,applied,F,actual.energy(q,dq),U,power,actual.b*(dq@dq),np.any(np.abs(tau)>20)])
        mujoco.mj_step(model,data)
        if not np.all(np.isfinite(data.qpos)):raise RuntimeError('Неограниченное состояние')
    if viewer:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(model,data) as window:
            while window.is_running() and data.time<duration-1e-10:
                start=time.perf_counter()
                with window.lock():tick()
                window.sync();time.sleep(max(0.,step-(time.perf_counter()-start)))
    else:
        while data.time<duration-1e-10:tick()
    return np.asarray(rows)


def metrics(rows):
    error=np.linalg.norm(rows[:,7:10]-rows[:,10:13],axis=1)
    return {'rms_tip_mm':float(1000*np.sqrt(np.mean(error**2))), 'max_tip_mm':float(1000*np.max(error)),
            'saturation_percent':float(100*np.mean(rows[:,-1]))}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--method',choices=(*METHODS,'all'),default='all');p.add_argument('--viewer',action='store_true')
    p.add_argument('--force',action='store_true');p.add_argument('--duration',type=float,default=8.);p.add_argument('--step',type=float,default=.002)
    p.add_argument('--model-mass-scale',type=float,default=1.);p.add_argument('--out',type=Path,default=Path(__file__).parent/'results')
    args=p.parse_args()
    if min(args.duration,args.step,args.model_mass_scale)<=0:p.error('Параметры должны быть положительными')
    if args.viewer and args.method=='all':p.error('Для viewer выберите один метод')
    args.out.mkdir(parents=True,exist_ok=True);datasets={};reports={}
    for method in METHODS if args.method=='all' else [args.method]:
        rows=run(method,args.duration,args.force,args.viewer,args.step,args.model_mass_scale)
        datasets[method]=rows;reports[method]=metrics(rows)
        np.savetxt(args.out/f'{method}.csv',rows,delimiter=',',header=','.join(COLUMNS),comments='')
        print(method,reports[method])
    (args.out/'metrics.json').write_text(json.dumps({'settings':vars(args)|{'out':str(args.out)},'metrics':reports},indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    for method,rows in datasets.items():
        axs[0,0].plot(rows[:,0],rows[:,2],label=method)
        axs[0,1].plot(rows[:,0],1000*np.linalg.norm(rows[:,7:10]-rows[:,10:13],axis=1),label=method)
        axs[1,0].plot(rows[:,0],rows[:,14],label=method)
        axs[1,1].plot(rows[:,0],rows[:,19]-rows[0,19],label=method)
    for ax,title,unit in zip(axs.flat,['Shoulder elevation q2','Tip error to each task target','Motor torque 2','Mechanical energy change'],['rad','mm','Nm','J']):
        ax.set(title=title,xlabel='t [s]',ylabel=unit);ax.grid(alpha=.3);ax.legend()
    fig.savefig(args.out/'comparison.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
