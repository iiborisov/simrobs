"""Слежение 2R за восьмёркой: суставной PD и PD с компенсацией тяжести."""
import argparse
import csv
from contextlib import nullcontext
from pathlib import Path
import time

import mujoco
import numpy as np

if __package__:
    from .robot_math import P, gravity, reference
else:
    from robot_math import P, gravity, reference

HERE = Path(__file__).resolve().parent


def run(controller='pd-gravity', duration=14.0, period=6.0, kp=(100., 60.), kd=(20., 12.), viewer=False):
    model = mujoco.MjModel.from_xml_path(str(HERE/'robot.xml'))
    data = mujoco.MjData(model)
    tip = model.site('tip').id
    target = model.site('target').id
    _, _, q0, dq0 = reference(0., period)
    data.qpos[:] = q0
    data.qvel[:] = dq0
    mujoco.mj_forward(model, data)
    records = []
    if viewer:
        import mujoco.viewer as mjviewer
        context = mjviewer.launch_passive(model, data)
    else:
        context = nullcontext(None)
    with context as window:
        if window is not None:
            window.cam.lookat[:] = [0.3, 0, 0.5]
            window.cam.distance = 1.6
            window.cam.azimuth = 90
            window.cam.elevation = -15
        count = int(np.ceil(duration/model.opt.timestep))
        for _ in range(count):
            if window is not None and not window.is_running():
                break
            wall_start = time.perf_counter()
            with window.lock() if window is not None else nullcontext():
                t = float(data.time)
                pos, _, q_ref, dq_ref = reference(t, period)
                # q_ref и q идут по одной непрерывной ветви вдали от ±pi.
                tau = np.asarray(kp)*(q_ref-data.qpos)+np.asarray(kd)*(dq_ref-data.qvel)
                if controller == 'pd-gravity':
                    tau += gravity(data.qpos)
                data.ctrl[:] = np.clip(tau, model.actuator_ctrlrange[:,0], model.actuator_ctrlrange[:,1])
                model.site_pos[target] = [pos[0], 0, pos[1]+P.base_height]
                # После предыдущего mj_step производные поля могли остаться от старого
                # состояния. Обновляем их перед чтением tip и логированием.
                mujoco.mj_forward(model, data)
                actual = data.site_xpos[tip, [0,2]]-np.array([0., P.base_height])
                records.append(np.r_[t, data.qpos, data.qvel, q_ref, dq_ref, actual, pos, data.ctrl,
                                        np.linalg.norm(actual-pos), np.any(np.abs(tau-data.ctrl)>1e-10)])
                mujoco.mj_step(model, data)
                if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                    raise ArithmeticError('Симуляция потеряла численную устойчивость.')
            if window is not None:
                window.sync()
                time.sleep(max(0., model.opt.timestep-(time.perf_counter()-wall_start)))
    if not records:
        raise RuntimeError('Нет данных: окно закрыто до первого шага.')
    return np.array(records)


def save_results(results, output, period):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True, exist_ok=True)
    labels = {'pd': 'PD', 'pd-gravity': 'PD + G(q)'}
    headers = ['t_s','q1_rad','q2_rad','dq1_rad_s','dq2_rad_s',
               'q1_ref_rad','q2_ref_rad','dq1_ref_rad_s','dq2_ref_rad_s',
               'x_m','z_m','x_ref_m','z_ref_m','tau1_Nm','tau2_Nm','error_m','saturated']
    fig, axes = plt.subplots(2,2,figsize=(12,9))
    dense = np.array([reference(t, period)[0] for t in np.linspace(2,2+period,1000)])
    axes[0,0].plot(dense[:,0],dense[:,1],'k--',label='Заданная восьмёрка')
    for controller, rows in results.items():
        with (output/f'{controller}.csv').open('w',newline='') as stream:
            writer=csv.writer(stream); writer.writerow(headers); writer.writerows(rows)
        axes[0,0].plot(rows[:,9],rows[:,10],label=labels[controller])
        axes[0,1].plot(rows[:,0],1000*rows[:,15],label=labels[controller])
        for joint in range(2):
            axes[1,0].plot(rows[:,0],rows[:,1+joint]-rows[:,5+joint],label=f'{labels[controller]}, q{joint+1}')
            axes[1,1].plot(rows[:,0],rows[:,13+joint],label=f'{labels[controller]}, τ{joint+1}')
        err=rows[:,15]
        print(f'{controller}: RMS={1000*np.sqrt(np.mean(err**2)):.3f} мм, '
              f'max={1000*np.max(err):.3f} мм, насыщение={100*np.mean(rows[:,16]):.2f}%')
    axes[0,0].set(xlabel='x относительно плеча, м',ylabel='z относительно плеча, м',title='Траектория tip')
    axes[0,0].set_aspect('equal',adjustable='box')
    axes[0,1].set(xlabel='t, с',ylabel='Ошибка tip, мм')
    axes[1,0].set(xlabel='t, с',ylabel='q − q_ref, рад')
    axes[1,1].set(xlabel='t, с',ylabel='Момент привода, Н·м')
    for axis in axes.flat:
        axis.grid(True); axis.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(output/'tracking.png',dpi=160); plt.close(fig)
    print(f'CSV и график: {output.resolve()}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller',choices=['pd','pd-gravity','compare'],default='compare')
    parser.add_argument('--viewer',action='store_true',help='Окно симуляции: выбрать один регулятор')
    parser.add_argument('--duration',type=float,default=14.)
    parser.add_argument('--period',type=float,default=6.)
    parser.add_argument('--kp',type=float,nargs=2,default=[100.,60.])
    parser.add_argument('--kd',type=float,nargs=2,default=[20.,12.])
    parser.add_argument('--output',type=Path,default=HERE/'results')
    args=parser.parse_args()
    if not np.isfinite([args.duration,args.period,*args.kp,*args.kd]).all() or min(args.duration,args.period)<=0 or min(args.kp)<0 or min(args.kd)<0:
        parser.error('Длительность и период должны быть >0; усиления >=0; все значения конечны.')
    if args.viewer and args.controller=='compare':
        parser.error('Для --viewer выберите --controller pd или pd-gravity.')
    controllers=['pd','pd-gravity'] if args.controller=='compare' else [args.controller]
    results={name:run(name,args.duration,args.period,args.kp,args.kd,args.viewer) for name in controllers}
    save_results(results,args.output,args.period)


if __name__=='__main__':
    main()
