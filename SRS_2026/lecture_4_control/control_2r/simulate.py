"""MuJoCo: сравнение computed torque, impedance и admittance."""
from pathlib import Path
import argparse
import json
import time
import numpy as np
import mujoco
if __package__:
    from .controllers import Controller, reference, joint_reference, external_force
    from .robot_math import P, forward_kinematics
else:
    from controllers import Controller, reference, joint_reference, external_force
    from robot_math import P, forward_kinematics

METHODS = ('computed-torque', 'impedance', 'admittance')
COLUMNS = 't q1 q2 dq1 dq2 x z xd zd xc zc fx fz tau1 tau2 saturated'.split()


def run(method, task='tracking', duration=10., force=False, viewer=False,
        stiffness=120., damping=22., virtual_mass=1.5, step=.002):
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name('robot.xml')))
    model.opt.timestep = step
    data = mujoco.MjData(model)
    data.qpos[:] = joint_reference(*reference(0, task))[0]
    mujoco.mj_forward(model, data)
    tip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'tip')
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'target')
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link2')
    controller = Controller(method, stiffness, damping, virtual_mass)
    rows = []
    def tick():
        mujoco.mj_forward(model, data)
        t = data.time
        q, dq = data.qpos.copy(), data.qvel.copy()
        f = external_force(t, force)
        tau, pc = controller.command(q, dq, t, f, task)
        applied = np.clip(tau, -35, 35)
        data.ctrl[:] = applied
        data.qfrc_applied[:] = 0
        # Мировые сила и точка; MuJoCo вычисляет обобщённую силу J^T F.
        mujoco.mj_applyFT(model, data, np.array([f[0], 0., f[1]]),
                          np.zeros(3), data.site_xpos[tip], body, data.qfrc_applied)
        pos = data.site_xpos[tip, [0, 2]]-np.array([0., P.base_height])
        pd = reference(t, task)[0]
        model.site_pos[target] = [pc[0], 0., pc[1]+P.base_height]
        rows.append(np.r_[t, q, dq, pos, pd, pc, f, applied, np.any(abs(tau)>35)])
        controller.advance(t, step, force)
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)):
            raise RuntimeError('Неограниченное состояние')
    if viewer:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(model, data) as window:
            while window.is_running() and data.time < duration-1e-10:
                started = time.perf_counter()
                with window.lock():
                    tick()
                window.sync()
                time.sleep(max(0., step-(time.perf_counter()-started)))
    else:
        while data.time < duration-1e-10:
            tick()
    return np.asarray(rows)


def summarize(rows):
    e = np.linalg.norm(rows[:,5:7]-rows[:,7:9], axis=1)
    ec = np.linalg.norm(rows[:,5:7]-rows[:,9:11], axis=1)
    return {'rms_nominal_mm': float(1000*np.sqrt(np.mean(e**2))),
            'max_nominal_mm': float(1000*np.max(e)),
            'rms_command_mm': float(1000*np.sqrt(np.mean(ec**2))),
            'saturation_percent': float(100*np.mean(rows[:,-1]))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--method', choices=(*METHODS, 'all'), default='all')
    parser.add_argument('--task', choices=('tracking','interaction'), default='tracking')
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--step', type=float, default=.002)
    parser.add_argument('--stiffness', type=float, default=120.)
    parser.add_argument('--damping', type=float, default=22.)
    parser.add_argument('--virtual-mass', type=float, default=1.5)
    parser.add_argument('--out', type=Path, default=Path(__file__).parent/'results')
    args = parser.parse_args()
    if min(args.duration, args.step, args.stiffness, args.damping, args.virtual_mass) <= 0:
        parser.error('Численные параметры должны быть положительными')
    if args.viewer and args.method == 'all':
        parser.error('Для viewer выберите один --method')
    args.out.mkdir(parents=True, exist_ok=True)
    datasets, metrics = {}, {}
    for method in METHODS if args.method=='all' else [args.method]:
        rows = run(method, args.task, args.duration, args.force, args.viewer,
                   args.stiffness, args.damping, args.virtual_mass, args.step)
        datasets[method] = rows
        metrics[method] = summarize(rows)
        np.savetxt(args.out/f'{method}.csv', rows, delimiter=',', header=','.join(COLUMNS), comments='')
        print(method, json.dumps(metrics[method], ensure_ascii=False))
    (args.out/'metrics.json').write_text(json.dumps({'settings': vars(args)|{'out':str(args.out)}, 'metrics':metrics}, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12,8), constrained_layout=True)
    for method, rows in datasets.items():
        axes[0,0].plot(rows[:,5], rows[:,6], label=method)
        axes[0,1].plot(rows[:,0], 1000*(rows[:,5]-rows[:,7]), label=method)
        axes[1,0].plot(rows[:,0], rows[:,13], label=method)
    rows = next(iter(datasets.values()))
    axes[0,0].plot(rows[:,7],rows[:,8],'k--',label='nominal')
    axes[1,1].plot(rows[:,0],rows[:,11],color='#7b263d')
    for ax,title,xlabel,ylabel in zip(axes.flat,
         ['Tip trajectory','Deviation from nominal','Shoulder torque','External force'],
         ['x [m]','t [s]','t [s]','t [s]'],['z [m]','x-xd [mm]','tau1 [Nm]','Fx [N]']):
        ax.set(title=title,xlabel=xlabel,ylabel=ylabel); ax.grid(True,alpha=.3)
    axes[0,0].axis('equal')
    for ax in list(axes.flat)[:3]: ax.legend()
    fig.savefig(args.out/'comparison.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
