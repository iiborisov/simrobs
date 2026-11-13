"""Три интерфейса управления. Все позиции p=(x,z) отсчитываются от плеча."""
import numpy as np
if __package__:
    from .robot_math import P, inverse_kinematics, jacobian, forward_kinematics, inverse_dynamics, gravity
else:
    from robot_math import P, inverse_kinematics, jacobian, forward_kinematics, inverse_dynamics, gravity


def jacobian_dot(q, dq):
    q1, q2 = q
    w1, w12 = dq[0], sum(dq)
    s1, c1, s12, c12 = np.sin(q1), np.cos(q1), np.sin(q1+q2), np.cos(q1+q2)
    return np.array([[-P.l1*c1*w1-P.l2*c12*w12, -P.l2*c12*w12],
                     [-P.l1*s1*w1-P.l2*s12*w12, -P.l2*s12*w12]])


def reference(t, task='tracking'):
    """p, dp, ddp; разгон восьмёрки C², после 2 с постоянная частота."""
    if task == 'interaction':
        return np.array([.4, .2]), np.zeros(2), np.zeros(2)
    omega, ramp = 2*np.pi/6, 2.
    if t < ramp:
        s = t/ramp
        a = omega*ramp*(s**3-.5*s**4)
        da = omega*(3*s*s-2*s**3)
        dda = omega/ramp*(6*s-6*s*s)
    else:
        a, da, dda = omega*(t-ramp/2), omega, 0.
    pos = np.array([.4+.12*np.sin(a), .2+.10*np.sin(2*a)])
    tangent = np.array([.12*np.cos(a), .20*np.cos(2*a)])
    curvature = np.array([-.12*np.sin(a), -.40*np.sin(2*a)])
    return pos, tangent*da, curvature*da**2+tangent*dda


def joint_reference(pos, vel, acc):
    q = inverse_kinematics(pos)
    dq = np.linalg.solve(jacobian(q), vel)
    ddq = np.linalg.solve(jacobian(q), acc-jacobian_dot(q, dq)@dq)
    return q, dq, ddq


def external_force(t, enabled=True):
    """Гладкий толчок 4 Н по +x между 3 и 5 с; сила среды НА робот."""
    return np.array([4*np.sin(np.pi*(t-3)/2)**2 if enabled and 3<t<5 else 0., 0.])


class Controller:
    def __init__(self, method, stiffness=120., damping=22., virtual_mass=1.5):
        if method not in ('computed-torque', 'impedance', 'admittance'):
            raise ValueError(method)
        if min(stiffness, damping, virtual_mass) <= 0:
            raise ValueError('Параметры должны быть положительными')
        self.method = method
        self.k, self.d, self.m = stiffness, damping, virtual_mass
        self.offset = np.zeros(4)  # delta_x, delta_z, delta_vx, delta_vz

    def command(self, q, dq, t, measured_force, task='tracking'):
        pd, vd, ad = reference(t, task)
        if self.method == 'impedance':
            # Виртуальные пружина и демпфер + компенсация тяжести.
            # НЕ задаёт произвольную декартову матрицу инерции.
            force = self.k*(pd-forward_kinematics(q))+self.d*(vd-jacobian(q)@dq)
            return gravity(q)+jacobian(q).T@force, pd
        pc, vc, ac = pd.copy(), vd.copy(), ad.copy()
        if self.method == 'admittance':
            delta, velocity = self.offset[:2], self.offset[2:]
            acceleration = (measured_force-self.d*velocity-self.k*delta)/self.m
            pc, vc, ac = pd+delta, vd+velocity, ad+acceleration
        qr, dqr, ddqr = joint_reference(pc, vc, ac)
        # Kp [1/s²], Kd [1/s]; omega_n=35 rad/s, zeta=1.
        acceleration = ddqr+70*(dqr-dq)+1225*(qr-q)
        return inverse_dynamics(q, dq, acceleration), pc

    def advance(self, t, dt, enabled):
        """Внешний admittance-фильтр: классический RK4, не интегратор MuJoCo."""
        if self.method != 'admittance':
            return
        def rhs(time, state):
            return np.r_[state[2:], (external_force(time, enabled)-self.d*state[2:]-self.k*state[:2])/self.m]
        y = self.offset
        k1 = rhs(t, y)
        k2 = rhs(t+dt/2, y+dt*k1/2)
        k3 = rhs(t+dt/2, y+dt*k2/2)
        k4 = rhs(t+dt, y+dt*k3)
        self.offset = y+dt*(k1+2*k2+2*k3+k4)/6
