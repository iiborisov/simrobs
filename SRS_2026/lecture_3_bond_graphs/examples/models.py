"""Три модели, выведенные из bond graphs. Состояния — переменные накопителей."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class MassSpringDamper:
    m: float = 1.0
    k: float = 20.0
    b: float = 0.5
    g: float = 9.8
    u: float = 0.0
    state_names = ('x_m', 'p_kg_m_s')
    title = 'Вертикальная масса–пружина–демпфер'
    duration = 10.0

    def initial(self):
        return np.array([0.5, 0.0])

    def rhs(self, t, state):
        x, momentum = state
        v = momentum/self.m
        # 1-узел: усилие I = усилие Se − усилие C − усилие R.
        return np.array([v, self.u-self.m*self.g-self.k*x-self.b*v])

    def gradient(self, state):
        x, momentum = state
        return np.array([self.k*x, momentum/self.m])

    def energy(self, state):
        x, momentum = state
        # Гравитация здесь внешний Se, а не часть энергии накопителей H.
        return 0.5*self.k*x*x + 0.5*momentum*momentum/self.m

    def ports(self, state):
        x, momentum = state
        v = momentum/self.m
        source = self.u-self.m*self.g
        return {'v_m_s': v, 'effort_Se_N': source, 'effort_C_N': self.k*x,
                'effort_R_N': self.b*v, 'power_in_W': source*v,
                'power_loss_W': self.b*v*v,
                'mechanical_energy_J': self.energy(state)+self.m*self.g*x}


@dataclass(frozen=True)
class SeriesRLC:
    L: float = 0.5
    C: float = 0.02
    R: float = 2.0
    U: float = 5.0
    state_names = ('charge_C', 'flux_Wb')
    title = 'Последовательная RLC-цепь'
    duration = 4.0

    def initial(self):
        return np.zeros(2)

    def rhs(self, t, state):
        charge, flux = state
        current = flux/self.L
        return np.array([current, self.U-self.R*current-charge/self.C])

    def gradient(self, state):
        charge, flux = state
        return np.array([charge/self.C, flux/self.L])

    def energy(self, state):
        charge, flux = state
        return 0.5*charge*charge/self.C+0.5*flux*flux/self.L

    def ports(self, state):
        charge, flux = state
        current = flux/self.L
        return {'current_A': current, 'voltage_C_V': charge/self.C,
                'voltage_R_V': self.R*current, 'voltage_Se_V': self.U,
                'power_in_W': self.U*current, 'power_loss_W': self.R*current*current}


@dataclass(frozen=True)
class MotorGearSpring:
    L: float = 0.2
    R: float = 2.0
    K: float = 0.2  # В СИ численно Ke=Kt для идеального преобразователя GY.
    Jm: float = 0.01
    Jl: float = 0.05
    bm: float = 0.002
    bl: float = 0.02
    k: float = 0.8
    n: float = 5.0  # omega_motor = n * omega_load.
    U: float = 6.0
    state_names = ('flux_Wb', 'p_equivalent_kg_m2_s', 'theta_load_rad')
    title = 'DC-мотор, идеальный редуктор и упругая нагрузка'
    duration = 12.0

    @property
    def J(self):
        return self.Jm+self.Jl/self.n**2

    @property
    def b(self):
        return self.bm+self.bl/self.n**2

    def initial(self):
        return np.zeros(3)

    def rhs(self, t, state):
        flux, momentum, theta = state
        current, omega = flux/self.L, momentum/self.J
        # GY: e_back=K*omega, torque_em=K*current.
        # TF: omega_load=omega/n, torque_motor_side=k*theta/n.
        return np.array([self.U-self.R*current-self.K*omega,
                         self.K*current-self.b*omega-self.k*theta/self.n,
                         omega/self.n])

    def gradient(self, state):
        flux, momentum, theta = state
        return np.array([flux/self.L, momentum/self.J, self.k*theta])

    def energy(self, state):
        flux, momentum, theta = state
        return 0.5*flux*flux/self.L+0.5*momentum*momentum/self.J+0.5*self.k*theta*theta

    def ports(self, state):
        flux, momentum, theta = state
        current, omega = flux/self.L, momentum/self.J
        alpha = self.rhs(0, state)[1]/self.J
        omega_load = omega/self.n
        # Через TF проходит момент нагрузки, а не весь электромагнитный момент!
        torque_load = self.Jl*alpha/self.n+self.bl*omega_load+self.k*theta
        torque_in = torque_load/self.n
        return {'current_A': current, 'omega_motor_rad_s': omega,
                'omega_load_rad_s': omega_load, 'back_emf_V': self.K*omega,
                'torque_em_Nm': self.K*current, 'torque_TF_in_Nm': torque_in,
                'torque_TF_out_Nm': torque_load,
                'power_GY_electric_W': self.K*omega*current,
                'power_GY_mechanical_W': self.K*current*omega,
                'power_TF_in_W': torque_in*omega,
                'power_TF_out_W': torque_load*omega_load,
                'power_in_W': self.U*current,
                'power_loss_W': self.R*current*current+self.b*omega*omega}


MODELS = {'msd': MassSpringDamper, 'rlc': SeriesRLC, 'motor': MotorGearSpring}


def rk4_step(fun, t, state, h):
    k1 = fun(t, state)
    k2 = fun(t+h/2, state+h*k1/2)
    k3 = fun(t+h/2, state+h*k2/2)
    k4 = fun(t+h, state+h*k3)
    return state+h*(k1+2*k2+2*k3+k4)/6


def simulate(model, duration=None, step=0.001):
    """Одновременно интегрируем состояния, работу источника и потери.

    Work и dissipation интегрируются тем же RK4, а не последующей
    прямоугольной квадратурой: это позволяет оценить ошибку баланса.
    """
    duration = model.duration if duration is None else duration
    if not np.isfinite([duration, step]).all() or min(duration, step) <= 0:
        raise ValueError('Длительность и шаг должны быть конечными и положительными.')
    count = int(np.ceil(duration/step))
    if count > 1_000_000:
        raise ValueError('Слишком много шагов; увеличьте шаг или сократите время.')
    initial = model.initial()
    size = len(initial)
    initial_energy = model.energy(initial)
    times = [0.0]
    values = [np.r_[initial, 0., 0.]]

    def augmented(t, state):
        physical = state[:size]
        ports = model.ports(physical)
        return np.r_[model.rhs(t, physical), ports['power_in_W'], ports['power_loss_W']]

    for n in range(count):
        target = min((n+1)*step, duration)
        state = rk4_step(augmented, times[-1], values[-1], target-times[-1])
        if not np.isfinite(state).all():
            raise ArithmeticError('Численная неустойчивость: уменьшите шаг.')
        times.append(target)
        values.append(state)
    rows = []
    for t, full in zip(times, values):
        state, work, dissipated = full[:size], full[-2], full[-1]
        H = model.energy(state)
        row = {'time_s': t, **dict(zip(model.state_names, state)), **model.ports(state),
               'H_J': H, 'work_in_J': work, 'dissipated_J': dissipated,
               'balance_residual_J': H-initial_energy-work+dissipated}
        rows.append(row)
    return rows
