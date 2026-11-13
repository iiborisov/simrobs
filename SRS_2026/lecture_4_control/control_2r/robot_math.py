"""Кинематика и аналитическая динамика того же 2R, что задан в robot.xml."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Parameters:
    l1: float = 0.4
    l2: float = 0.4
    m1: float = 2.0
    m2: float = 2.0
    r1: float = 0.2  # Расстояние до центра масс первого звена.
    r2: float = 0.2
    I1: float = 2*0.4**2/12  # Инерция вокруг центра масс, ось поперёк плоскости.
    I2: float = 2*0.4**2/12
    g: float = 9.81
    damping: float = 0.05
    base_height: float = 0.3


P = Parameters()


def forward_kinematics(q, p=P):
    """Положение tip (x,z) относительно плеча, а не мирового начала."""
    q1, q2 = q
    return np.array([p.l1*np.cos(q1)+p.l2*np.cos(q1+q2),
                     p.l1*np.sin(q1)+p.l2*np.sin(q1+q2)])


def inverse_kinematics(position, elbow=1, p=P):
    """Одна выбранная ветвь IK; недостижимая цель вызывает ошибку."""
    if elbow not in (-1, 1):
        raise ValueError('elbow должен быть -1 или +1')
    x, z = position
    c2 = (x*x+z*z-p.l1*p.l1-p.l2*p.l2)/(2*p.l1*p.l2)
    if abs(c2) > 1+1e-12:
        raise ValueError(f'Недостижимая точка: {position}')
    c2 = np.clip(c2, -1, 1)  # Только погрешность округления у границы.
    s2 = elbow*np.sqrt(max(0.0, 1-c2*c2))
    q2 = np.arctan2(s2, c2)
    q1 = np.arctan2(z, x)-np.arctan2(p.l2*s2, p.l1+p.l2*c2)
    return np.array([q1, q2])


def jacobian(q, p=P):
    q1, q2 = q
    return np.array([
        [-p.l1*np.sin(q1)-p.l2*np.sin(q1+q2), -p.l2*np.sin(q1+q2)],
        [p.l1*np.cos(q1)+p.l2*np.cos(q1+q2), p.l2*np.cos(q1+q2)],
    ])


def mass_matrix(q, p=P):
    c2 = np.cos(q[1])
    a = p.I1+p.I2+p.m1*p.r1**2+p.m2*(p.l1**2+p.r2**2)
    d = p.I2+p.m2*p.r2**2
    beta = p.m2*p.l1*p.r2
    return np.array([[a+2*beta*c2, d+beta*c2], [d+beta*c2, d]])


def coriolis(q, dq, p=P):
    """Вектор c=C*dq, не сама матрица C."""
    v1, v2 = dq
    h = p.m2*p.l1*p.r2*np.sin(q[1])
    return np.array([-h*(2*v1*v2+v2*v2), h*v1*v1])


def gravity(q, p=P):
    """G=dV/dq в левой части M*ddq+c+G+D*dq=tau."""
    q1, q2 = q
    g2 = p.m2*p.g*p.r2*np.cos(q1+q2)
    return np.array([(p.m1*p.r1+p.m2*p.l1)*p.g*np.cos(q1)+g2, g2])


def inverse_dynamics(q, dq, ddq, p=P):
    return mass_matrix(q,p)@ddq+coriolis(q,dq,p)+gravity(q,p)+p.damping*np.asarray(dq)


def forward_dynamics(q, dq, tau, p=P):
    return np.linalg.solve(mass_matrix(q,p), np.asarray(tau)-coriolis(q,dq,p)-gravity(q,p)-p.damping*np.asarray(dq))
