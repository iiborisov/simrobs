"""Примеры bond graphs: состояния, мощности, накопленная работа и баланс энергии."""
import argparse
import csv
from pathlib import Path
import numpy as np

if __package__:
    from .models import MODELS, simulate
else:
    from models import MODELS, simulate


def plot_rows(name, model, rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'serif', 'axes.spines.top': False, 'axes.spines.right': False})
    colors = ('#7b263d', '#b65b20', '#477994')
    t = [r['time_s'] for r in rows]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    signals = {'msd': [('x_m', 'x, м'), ('v_m_s', 'v, м/с')],
               'rlc': [('current_A', 'i, А'), ('voltage_C_V', 'u_C, В')],
               'motor': [('current_A', 'i, А'), ('theta_load_rad', 'θ нагрузки, рад')]}[name]
    # Разные единицы не складываются на одной шкале.
    ax2 = axes[0, 0].twinx()
    for axis, (key, label), color in zip((axes[0, 0], ax2), signals, colors):
        axis.plot(t, [r[key] for r in rows], color=color, label=label)
        axis.set_ylabel(label, color=color)
    axes[0, 0].set_title('Физические переменные')
    for key, label, color in [('power_in_W', 'Мощность источника', colors[0]),
                              ('power_loss_W', 'Мощность потерь', colors[1])]:
        axes[0, 1].plot(t, [r[key] for r in rows], label=label, color=color)
    axes[0, 1].set_ylabel('Вт'); axes[0, 1].legend()
    for key, label, color in [('H_J', 'H накопителей', colors[0]),
                             ('work_in_J', 'Работа источника', colors[2]),
                             ('dissipated_J', 'Рассеянная энергия', colors[1])]:
        axes[1, 0].plot(t, [r[key] for r in rows], label=label, color=color)
    axes[1, 0].set_ylabel('Дж'); axes[1, 0].legend()
    axes[1, 1].plot(t, [r['balance_residual_J'] for r in rows], color=colors[0])
    axes[1, 1].set_title('H − H₀ − W_in + E_diss'); axes[1, 1].set_ylabel('Невязка, Дж')
    for axis in axes.flat:
        axis.set_xlabel('t, с'); axis.grid(True, alpha=0.25)
    fig.suptitle(model.title)
    fig.tight_layout(); fig.savefig(output/f'{name}.png', dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=[*MODELS, 'all'], default='all')
    parser.add_argument('--step', type=float, default=0.001)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--gear', type=float, default=5., help='n=omega_motor/omega_load')
    parser.add_argument('--plot', action='store_true', help='Сохранить PNG; нужен matplotlib')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent/'results')
    args = parser.parse_args()
    if not np.isfinite(args.gear) or args.gear <= 0:
        parser.error('Передаточное отношение должно быть конечным и > 0.')
    names = list(MODELS) if args.model == 'all' else [args.model]
    args.output.mkdir(parents=True, exist_ok=True)
    for name in names:
        model = MODELS[name](n=args.gear) if name == 'motor' else MODELS[name]()
        try:
            rows = simulate(model, args.duration, args.step)
        except (ValueError, ArithmeticError) as exc:
            parser.exit(1, f'Ошибка: {exc}\n')
        with (args.output/f'{name}.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        residual = max(abs(r['balance_residual_J']) for r in rows)
        print(f'{name}: max |невязка энергии| = {residual:.3e} Дж; '
              f'H(T)={rows[-1]["H_J"]:.6f} Дж, W_in={rows[-1]["work_in_J"]:.6f} Дж, '
              f'E_diss={rows[-1]["dissipated_J"]:.6f} Дж')
        if args.plot:
            plot_rows(name, model, rows, args.output)
    print('Результаты:', args.output.resolve())


if __name__ == '__main__':
    main()
