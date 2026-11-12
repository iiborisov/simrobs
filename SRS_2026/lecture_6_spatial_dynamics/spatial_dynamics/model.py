"""Пространственная динамика того же 3R: home-оси z,x,x, вертикаль +z.
Нотация: I^{c_i,i} — 6D инерция, J_i — якобиан, M — суставная инерция.
W — строка; массив tau в коде представляет столбец tau^T из конспекта.
"""
import numpy as np
if __package__:
    from .geometry import skew, adjoint, inverse, exp_se3, robot_definition, transform
else:
    from geometry import skew, adjoint, inverse, exp_se3, robot_definition, transform

L=np.array([.4,.35,.25]);MASSES=np.array([2.,1.5,1.]);RADIUS=.025


def ad(T):
    w,v=T[:3],T[3:]
    return np.block([[skew(w),np.zeros((3,3))],[skew(v),skew(w)]])


class Robot:
    def __init__(self,mass_scale=1.,damping=.06):
        if mass_scale<=0 or damping<0:raise ValueError('Некорректные параметры')
        self.m=MASSES*mass_scale;self.b=damping;self.g=9.81
        transverse=self.m*(3*RADIUS**2+L**2)/12
        axial=.5*self.m*RADIUS**2
        self.J=np.array([[transverse[0],transverse[0],axial[0]],
                         [transverse[1],axial[1],transverse[1]],
                         [transverse[2],axial[2],transverse[2]]])

    def body_inertias(self):
        return [np.diag(np.r_[self.J[i],np.repeat(self.m[i],3)]) for i in range(3)]

    def mass(self,q):
        _,b,c=q;l1,l2,l3=L;r2,r3=l2/2,l3/2;m1,m2,m3=self.m
        a=b+c;rho=l2*np.cos(b)+r3*np.cos(a)
        H=(self.J[0,2]+m2*r2**2*np.cos(b)**2+self.J[1,1]*np.sin(b)**2
           +self.J[1,2]*np.cos(b)**2+m3*rho**2+self.J[2,1]*np.sin(a)**2+self.J[2,2]*np.cos(a)**2)
        D=self.J[2,0]+m3*r3**2;beta=m3*l2*r3
        return np.array([[H,0.,0.],[0.,self.J[1,0]+m2*r2**2+D+m3*l2**2+2*beta*np.cos(c),D+beta*np.cos(c)],
                         [0.,D+beta*np.cos(c),D]])

    def mass_derivatives(self,q):
        _,b,c=q;l1,l2,l3=L;r2,r3=l2/2,l3/2;_,m2,m3=self.m;a=b+c
        rho=l2*np.cos(b)+r3*np.cos(a)
        dH2=2*(self.J[1,1]-self.J[1,2]-m2*r2**2)*np.sin(b)*np.cos(b)+2*m3*rho*(-l2*np.sin(b)-r3*np.sin(a))+2*(self.J[2,1]-self.J[2,2])*np.sin(a)*np.cos(a)
        dH3=2*m3*rho*(-r3*np.sin(a))+2*(self.J[2,1]-self.J[2,2])*np.sin(a)*np.cos(a)
        h=m3*l2*r3*np.sin(c)
        d=np.zeros((3,3,3)) # d[k,i,j] = partial M_ij / partial q_k
        d[1,0,0]=dH2;d[2,0,0]=dH3;d[2,1,1]=-2*h;d[2,1,2]=d[2,2,1]=-h
        return d

    def coriolis_matrix(self,q,dq):
        d=self.mass_derivatives(q);C=np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                for k in range(3):C[i,j]+=.5*(d[k,i,j]+d[j,i,k]-d[i,k,j])*dq[k]
        return C

    def potential(self,q):
        _,b,c=q;l1,l2,l3=L;m1,m2,m3=self.m
        return self.g*(m1*l1/2+m2*(l1+l2/2*np.sin(b))+m3*(l1+l2*np.sin(b)+l3/2*np.sin(b+c)))

    def gravity(self,q):
        _,b,c=q;_,l2,l3=L;_,m2,m3=self.m
        g3=self.g*m3*l3/2*np.cos(b+c)
        return np.array([0.,self.g*(m2*l2/2+m3*l2)*np.cos(b)+g3,g3])

    def inverse_dynamics(self,q,dq,ddq):
        return self.mass(q)@ddq+self.coriolis_matrix(q,dq)@dq+self.gravity(q)+self.b*dq

    def forward_dynamics(self,q,dq,tau,external=None):
        force=np.zeros(3) if external is None else external
        return np.linalg.solve(self.mass(q),tau+force-self.coriolis_matrix(q,dq)@dq-self.gravity(q)-self.b*dq)

    def energy(self,q,dq):return .5*dq@self.mass(q)@dq+self.potential(q)

    def mass_from_energy(self,q):
        """Независимая сборка через CoM-якобианы и пространственные инерции."""
        axes,_=robot_definition(L);prefix=np.eye(4);Js=np.zeros((6,3));result=np.zeros((3,3))
        homes=[[0,0,L[0]/2],[0,L[1]/2,L[0]],[0,L[1]+L[2]/2,L[0]]]
        for i in range(3):
            Js[:,i]=adjoint(prefix)@axes[:,i]
            prefix=prefix@exp_se3(axes[:,i]*q[i])
            H_c_0=prefix@transform(o=homes[i]);Jb=adjoint(inverse(H_c_0))@Js
            result+=Jb.T@self.body_inertias()[i]@Jb
        return result


def kinematics(q):
    """R_4^0, o_4^0, J_omega и J_p; аналитический быстрый вариант."""
    a,b,c=q;l1,l2,l3=L;sa,ca=np.sin(a),np.cos(a);alpha=b+c
    Rz=np.array([[ca,-sa,0.],[sa,ca,0.],[0.,0.,1.]])
    Rx=np.array([[1.,0.,0.],[0.,np.cos(alpha),-np.sin(alpha)],[0.,np.sin(alpha),np.cos(alpha)]])
    rho=l2*np.cos(b)+l3*np.cos(alpha);z=l1+l2*np.sin(b)+l3*np.sin(alpha)
    p=np.array([-sa*rho,ca*rho,z])
    Jw=np.array([[0.,ca,ca],[0.,sa,sa],[1.,0.,0.]])
    dr2=-l2*np.sin(b)-l3*np.sin(alpha);dr3=-l3*np.sin(alpha)
    dz2=l2*np.cos(b)+l3*np.cos(alpha);dz3=l3*np.cos(alpha)
    Jp=np.array([[-ca*rho,-sa*dr2,-sa*dr3],[-sa*rho,ca*dr2,ca*dr3],[0.,dz2,dz3]])
    return Rz@Rx,p,Jw,Jp
