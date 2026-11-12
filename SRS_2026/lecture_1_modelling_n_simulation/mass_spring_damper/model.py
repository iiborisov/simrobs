"""Общая модель и три учебных интегратора. Только стандартная библиотека."""
from dataclasses import dataclass
from math import ceil, cos, exp, isfinite, sin, sqrt


@dataclass(frozen=True)
class Parameters:
    m: float = 1.0
    k: float = 20.0
    b: float = 0.5
    g: float = 9.8
    force: float = 0.0  # Постоянная внешняя сила, положительная вверх.

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.m, self.k, self.b, self.g, self.force)):
            raise ValueError('Параметры должны быть конечными.')
        if self.m <= 0 or self.k <= 0 or self.b < 0:
            raise ValueError('Нужны m > 0, k > 0, b >= 0.')

    @property
    def equilibrium(self):
        return (self.force - self.m * self.g) / self.k


def rhs(t, state, p):
    """y' = f(t, y), y = (положение, скорость). Вход здесь постоянен."""
    x, v = state
    mg = p.m * p.g  # Сила тяжести, Н; направлена вниз при оси x вверх.
    acceleration = (p.force - mg - p.b * v - p.k * x) / p.m
    return v, acceleration


def forward_euler(t, state, h, p):
    """Обе производные вычисляются по СТАРОМУ состоянию."""
    x, v = state
    dx, dv = rhs(t, state, p)
    return x + h * dx, v + h * dv


def backward_euler(t, state, h, p):
    """Решаем линейные неявные уравнения точно, без итераций.

    x_new = x + h*v_new
    m*v_new = m*v + h*(force - m*g - b*v_new - k*x_new)
    """
    x, v = state
    mg = p.m * p.g  # Сила тяжести входит со знаком минус.
    v_new = (p.m*v + h*(p.force - mg) - h*p.k*x) / (
        p.m + h*p.b + h*h*p.k
    )
    x_new = x + h*v_new
    return x_new, v_new


def rk4(t, state, h, p):
    """Классический явный Рунге–Кутта четвёртого порядка."""
    x, v = state
    k1 = rhs(t, (x, v), p)
    k2 = rhs(t + h/2, (x + h*k1[0]/2, v + h*k1[1]/2), p)
    k3 = rhs(t + h/2, (x + h*k2[0]/2, v + h*k2[1]/2), p)
    k4 = rhs(t + h, (x + h*k3[0], v + h*k3[1]), p)
    return (
        x + h*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0])/6,
        v + h*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])/6,
    )


METHODS = {'forward_euler': forward_euler, 'backward_euler': backward_euler, 'rk4': rk4}


def exact(t, initial, p):
    """Аналитическое решение при постоянной силе, включая все виды демпфирования."""
    z0 = initial[0] - p.equilibrium
    v0 = initial[1]
    alpha = p.b / (2*p.m)
    omega2 = p.k / p.m
    discriminant = alpha*alpha - omega2
    if abs(discriminant) <= 1e-12 * max(alpha*alpha, omega2):
        c = v0 + alpha*z0
        z = exp(-alpha*t)*(z0 + c*t)
        v = exp(-alpha*t)*(c - alpha*(z0 + c*t))
    elif discriminant < 0:
        omega = sqrt(-discriminant)
        c = (v0 + alpha*z0)/omega
        z = exp(-alpha*t)*(z0*cos(omega*t) + c*sin(omega*t))
        v = -alpha*z + exp(-alpha*t)*omega*(-z0*sin(omega*t) + c*cos(omega*t))
    else:
        # Произведение корней = omega2: так избегаем вычитания близких чисел.
        r2 = -alpha - sqrt(discriminant)
        r1 = omega2/r2
        c1 = (v0 - r2*z0)/(r1-r2)
        c2 = z0-c1
        z = c1*exp(r1*t) + c2*exp(r2*t)
        v = r1*c1*exp(r1*t) + r2*c2*exp(r2*t)
    return z + p.equilibrium, v


def energy(state, p):
    """Энергия относительно равновесия при постоянном входе.

    E' = -b*v**2. Это механическая энергия с учётом потенциала
    постоянной внешней силы, с точностью до аддитивной константы.
    """
    x, v = state
    return 0.5*p.m*v*v + 0.5*p.k*(x-p.equilibrium)**2


def simulate(method, p, initial, duration, step):
    if not isfinite(step) or not isfinite(duration) or step <= 0 or duration <= 0:
        raise ValueError('Шаг и длительность должны быть конечными и положительными.')
    if not all(isfinite(value) for value in initial):
        raise ValueError('Начальное состояние должно быть конечным.')
    count = ceil(duration/step)
    if count > 2_000_000:
        raise ValueError('Слишком много шагов: увеличьте шаг или сократите интервал.')
    times, states = [0.0], [tuple(initial)]
    for n in range(count):
        t_next = min((n+1)*step, duration)
        h = t_next-times[-1]  # Последний шаг укорочен, если нужно.
        state = METHODS[method](times[-1], states[-1], h, p)
        if not all(isfinite(value) for value in state):
            raise ArithmeticError(f'{method}: переполнение при t={t_next:g}; уменьшите шаг.')
        times.append(t_next)
        states.append(state)
    return times, states
