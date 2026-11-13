"""Физические проверки: локальная мощность, GY/TF, эквивалентность и сходимость."""
import numpy as np
if __package__:
    from .models import MassSpringDamper, SeriesRLC, MotorGearSpring, MODELS, simulate
else:
    from models import MassSpringDamper, SeriesRLC, MotorGearSpring, MODELS, simulate


def main():
    rng = np.random.default_rng(23)
    for name, factory in MODELS.items():
        model = factory()
        for _ in range(100):
            y = rng.uniform(-1, 1, len(model.initial()))
            ports = model.ports(y)
            derivative = model.gradient(y)@model.rhs(0, y)
            assert np.isclose(derivative, ports['power_in_W']-ports['power_loss_W'], atol=1e-10)
            assert ports['power_loss_W'] >= 0
            if name == 'motor':
                assert np.isclose(ports['power_GY_electric_W'], ports['power_GY_mechanical_W'])
                assert np.isclose(ports['power_TF_in_W'], ports['power_TF_out_W'])
        rows = simulate(model)
        scale = max(1., abs(rows[-1]['work_in_J']), rows[0]['H_J'])
        residual = max(abs(r['balance_residual_J']) for r in rows)
        assert residual/scale < 1e-8, (name, residual)
        print(name, 'баланс энергии и пассивность: OK; невязка', f'{residual:.3e}', 'Дж')
    msd = MassSpringDamper()
    x, momentum = .3, -.7
    v = momentum/msd.m
    assert np.allclose(msd.rhs(0, [x, momentum]), [v, -msd.m*msd.g-msd.k*x-msd.b*v])
    for model, state in [
        (msd, [-msd.m*msd.g/msd.k, 0]),
        (SeriesRLC(), [.02*5, 0]),
        (MotorGearSpring(), [.2*6/2, 0, 5*.2*6/(2*.8)]),
    ]:
        assert np.max(np.abs(model.rhs(0, state))) < 1e-12
    # Сходимость RK4 по независимому мелкому расчёту.
    model = SeriesRLC()
    ref = simulate(model, duration=1., step=.0005)[-1]
    target = np.array([ref[k] for k in model.state_names])
    errors = []
    for h in [.01, .005, .0025]:
        row = simulate(model, duration=1., step=h)[-1]
        errors.append(np.linalg.norm(np.array([row[k] for k in model.state_names])-target))
    ratios = np.array(errors[:-1])/errors[1:]
    assert np.all((ratios > 13) & (ratios < 19)), ratios
    print('Равновесия, механический баланс сил, сходимость RK4: OK; отношения ошибок', ratios)


if __name__ == '__main__':
    main()
