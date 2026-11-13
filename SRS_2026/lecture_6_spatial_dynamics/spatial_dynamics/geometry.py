"""Нотация SRS: H_i^j переводит i -> j, T=(w;v), W=(tau,F) — строка.
Имена H_i_j, T_i_k_j читаются в том же порядке, что индексы в конспекте.
"""
import numpy as np


def skew(a):
    """Тильда: skew(a) @ b == cross(a,b)."""
    x, y, z = np.asarray(a, dtype=float)
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def unskew(A):
    return np.array([A[2,1], A[0,2], A[1,0]])


def hat(T):
    result = np.zeros((4,4))
    result[:3,:3], result[:3,3] = skew(T[:3]), T[3:]
    return result


def vee(A):
    return np.r_[unskew(A[:3,:3]), A[:3,3]]


def coefficients(theta):
    """Устойчивые sin(theta)/theta и коэффициенты экспоненты."""
    if abs(theta) < 1e-4:
        s = theta*theta
        return 1-s/6+s*s/120, .5-s/24+s*s/720, 1/6-s/120+s*s/5040
    return np.sin(theta)/theta, (1-np.cos(theta))/theta**2, (theta-np.sin(theta))/theta**3


def exp_so3(phi):
    """phi=w_hat*q — интегрированный вектор поворота, а не угловая скорость."""
    phi = np.asarray(phi, dtype=float)
    A, B, _ = coefficients(np.linalg.norm(phi))
    K = skew(phi)
    return np.eye(3)+A*K+B*K@K


def transform(R=None, o=None):
    H = np.eye(4)
    if R is not None: H[:3,:3] = R
    if o is not None: H[:3,3] = o
    return H


def inverse(H):
    R, o = H[:3,:3], H[:3,3]
    return transform(R.T, -R.T@o)


def exp_se3(X):
    """exp(tilde(X)); X=T*dt или нормированная ось * перемещение."""
    X = np.asarray(X, dtype=float)
    phi, rho = X[:3], X[3:]
    _, B, C = coefficients(np.linalg.norm(phi))
    K = skew(phi)
    V = np.eye(3)+B*K+C*K@K
    return transform(exp_so3(phi), V@rho)


def adjoint(H_i_j):
    R, o = H_i_j[:3,:3], H_i_j[:3,3]
    Ad = np.zeros((6,6))
    Ad[:3,:3], Ad[3:,3:], Ad[3:,:3] = R, R, skew(o)@R
    return Ad


def change_twist(H_j_k, T_i_j_j):
    return adjoint(H_j_k)@T_i_j_j


def change_wrench(H_j_k, W_i_j):
    """W_i^k = W_i^j Ad_(H_k^j); W хранится одномерным массивом-строкой."""
    return np.asarray(W_i_j)@adjoint(inverse(H_j_k))


LENGTHS = np.array([.4,.35,.25])


def robot_definition(lengths=LENGTHS):
    """Оси z,x,x в home; r2=(0,0,l1), r3=(0,l2,l1)."""
    l1,l2,l3 = lengths
    directions = [[0,0,1],[1,0,0],[1,0,0]]
    points = [[0,0,0],[0,0,l1],[0,l2,l1]]
    axes = np.column_stack([np.r_[w,np.cross(r,w)] for w,r in zip(directions,points)])
    H_4_0_home = transform(o=[0,l2+l3,l1])
    return axes,H_4_0_home


def forward_kinematics(q, lengths=LENGTHS):
    axes,home = robot_definition(lengths)
    prefix = np.eye(4)
    for axis,angle in zip(axes.T,q): prefix = prefix@exp_se3(axis*angle)
    return prefix@home


def space_jacobian(q, lengths=LENGTHS):
    axes,_ = robot_definition(lengths)
    J = np.zeros((6,3))
    prefix = np.eye(4)
    for i in range(3):
        J[:,i] = adjoint(prefix)@axes[:,i]
        prefix = prefix@exp_se3(axes[:,i]*q[i])
    return J


def body_jacobian(q, lengths=LENGTHS):
    return adjoint(inverse(forward_kinematics(q,lengths)))@space_jacobian(q,lengths)


def point_jacobian(q, lengths=LENGTHS):
    """Скорость начала Psi4; нижние три строки Js — НЕ этот якобиан."""
    H = forward_kinematics(q,lengths)
    return np.hstack([-skew(H[:3,3]), np.eye(3)])@space_jacobian(q,lengths)


def direct_chain(q, lengths=LENGTHS):
    """Независимая от PoE композиция локальных преобразований."""
    l1,l2,l3 = lengths
    return (transform(exp_so3([0,0,q[0]]))@transform(o=[0,0,l1])
            @transform(exp_so3([q[1],0,0]))@transform(o=[0,l2,0])
            @transform(exp_so3([q[2],0,0]))@transform(o=[0,l3,0]))
