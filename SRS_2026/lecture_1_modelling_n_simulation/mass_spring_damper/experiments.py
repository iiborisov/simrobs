"""Запуск одного метода или сравнение трёх методов с точным решением."""
import argparse
import csv
from pathlib import Path

if __package__:
    from .model import Parameters, energy, exact, simulate, METHODS
else:  # Прямой запуск файла, в том числе из IDE.
    from model import Parameters, energy, exact, simulate, METHODS

# notebook: буквально параметры из SRS/lecture_1/modeling_n_simulation.ipynb.
# Во всех сценариях включена гравитация.
# undamped/stiff: x0 = x_equilibrium + 1 м.
# undamped: энергия относительно равновесия должна сохраняться.
# stiff: собственные значения примерно -0.1 и -999.9 1/с.
SCENARIOS = {
    'notebook': (Parameters(g=9.8), (0.5, 0.0), 10.0),
    'undamped': (Parameters(k=4, b=0, g=9.8), (-1.45, 0.0), 20.0),
    'stiff': (Parameters(k=100, b=1000, g=9.8), (0.902, 0.0), 0.2),
}
LABELS = {'forward_euler': 'Прямой Эйлер', 'backward_euler': 'Обратный Эйлер', 'rk4': 'RK4'}


def plot_results(path, scenario, step, p, initial, duration, results):
    # Импорт отложен: расчёт и CSV не требуют сторонних пакетов.
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit('Для --plot установите зависимости: python3 -m pip install -r requirements.txt') from exc
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    reference_times = [duration*n/2000 for n in range(2001)]
    reference = [exact(t, initial, p) for t in reference_times]
    for axis, values in zip(axes, [
        [y[0] for y in reference], [y[1] for y in reference], [energy(y, p) for y in reference]
    ]):
        axis.plot(reference_times, values, 'k--', label='Точное', linewidth=2)
    for method, (times, states) in results.items():
        series = [[s[0] for s in states], [s[1] for s in states], [energy(s, p) for s in states]]
        for axis, values in zip(axes, series):
            axis.plot(times, values, label=LABELS[method], alpha=0.8)
    # symlog сохраняет видимость и малых значений, и численного разгона.
    axes[0].set_yscale('symlog', linthresh=max(abs(initial[0]-p.equilibrium), 0.01))
    axes[1].set_yscale('symlog', linthresh=1)
    axes[2].set_yscale('symlog', linthresh=max(energy(initial, p), 0.01))
    for axis, label in zip(axes, ['x, м (symlog)', 'v, м/с (symlog)', 'E_rel, Дж (symlog)']):
        axis.set_ylabel(label)
        axis.grid(True)
        axis.legend()
    axes[-1].set_xlabel('t, с')
    fig.suptitle(f'{scenario}: h={step:g} с; m={p.m:g}, k={p.k:g}, b={p.b:g}, g={p.g:g}')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main(single_method=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=[*SCENARIOS, 'all'], default='notebook')
    parser.add_argument('--steps', type=float, nargs='+', default=[0.1, 0.05, 0.01], help='Шаги в секундах')
    parser.add_argument('--duration', type=float, help='Длительность вместо значения сценария')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'results')
    parser.add_argument('--plot', action='store_true', help='Также сохранить PNG (нужен matplotlib)')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scenarios = SCENARIOS if args.scenario == 'all' else {args.scenario: SCENARIOS[args.scenario]}
    methods = [single_method] if single_method else list(METHODS)
    summary = []
    print('Сценарий    Метод              h, с      max|ошибка x|, м   max|ошибка v|, м/с   E_rel(T)/E_rel(0)')
    for scenario, (p, initial, default_duration) in scenarios.items():
        duration = args.duration if args.duration is not None else default_duration
        for step in args.steps:
            results = {}
            for method in methods:
                try:
                    times, states = simulate(method, p, initial, duration, step)
                    references = [exact(t, initial, p) for t in times]
                    energies = [energy(state, p) for state in states]
                    error_x = max(abs(s[0]-r[0]) for s, r in zip(states, references))
                    error_v = max(abs(s[1]-r[1]) for s, r in zip(states, references))
                    ratio = energies[-1]/energies[0] if energies[0] else float('nan')
                except (ArithmeticError, ValueError) as exc:
                    parser.exit(1, f'Ошибка: {exc}\n')
                summary.append([scenario, method, step, error_x, error_v, ratio])
                print(f'{scenario:11} {method:18} {step:8g} {error_x:18.6g} {error_v:21.6g} {ratio:13.6g}')
                file = args.output/f'{scenario}_{method}_h{step:g}.csv'
                with file.open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['t_s', 'x_m', 'v_m_s', 'x_exact_m', 'v_exact_m_s', 'energy_J', 'energy_exact_J'])
                    for t, state, ref, e in zip(times, states, references, energies):
                        writer.writerow([t, *state, *ref, e, energy(ref, p)])
                results[method] = times, states
            if args.plot:
                plot_results(args.output/f'{scenario}_h{step:g}.png', scenario, step, p, initial, duration, results)
    # Разные имена позволяют запускать отдельные методы без затирания чужой сводки.
    with (args.output/f'summary_{single_method or "all"}.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['scenario', 'method', 'h_s', 'max_error_x_m', 'max_error_v_m_s', 'energy_ratio'])
        writer.writerows(summary)
    print(f'Результаты сохранены: {args.output.resolve()}')


if __name__ == '__main__':
    main()
