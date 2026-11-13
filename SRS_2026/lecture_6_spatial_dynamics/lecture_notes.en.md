[Русский](lecture_notes.md) | **English**

# Lecture 6. Spatial Dynamics: From Rigid Bodies to 3R Control

**90 minutes.** Spatial inertia/momentum, Newton–Euler equations, energy-based robot dynamics, forward/inverse dynamics, and control.

[Presentation](presentation/index.en.html) · [Python/MuJoCo examples](spatial_dynamics/README.en.md) · [Slide schedule](presentation/TEACHING_PLAN.en.md) · [Previous lecture: spatial kinematics](../lecture_5_spatial_kinematics/lecture_notes.en.md).

Main sources: [Chapter 6, Dynamics](../../SRS/lecture_6/lecture_6.pdf), and [Chapter 7, Control](../../SRS/lecture_7/lecture_7.pdf). Additional source: Lynch and Park, *Modern Robotics*, Chapters 8 and 11. Notation is consistent with Lecture 5; formula and convention checks are discussed in §15.

## 1. What dynamics adds

Kinematics maps q to configuration H and velocity T. Dynamics relates motion to inertia, gravity, and applied forces. Unlike the previous lecture, MuJoCo now **integrates motion** through `mj_step`; the controller supplies torques without overwriting q each step.

Notation and conventions:

| Symbol | Meaning |
|---|---|
| Hᵢʲ | Coordinate transform from Ψᵢ to Ψⱼ |
| Tᵢᵏ˒ʲ=(w;v) | Twist column, angular part first |
| Wᵢᵏ=(τ F) | Wrench row: moment about Ψₖ's origin, then force |
| Iᵏ˒ⁱ | Spatial inertia of body i expressed in Ψₖ |
| Pᵏ˒ⁱ | Spatial momentum of body i, a **row** |
| K, P(q), L=K−P | Kinetic energy, potential energy, Lagrangian |
| τₐ | Actuator-torque row; equations use τₐᵀ |
| N(q) | Gravity vector ∂P/∂q, also G(q) in the source |
| C(q,q̇), c=Cq̇ | n×n matrix and corresponding n×1 vector |

Full indices are \(H_i^j,T_i^{k,j},I^{k,i},P^{k,i}\). Power is \(\mathcal P\) and homogeneous point \(\mathsf P\), distinct from momentum P and potential P(q). Python `tau` represents column τₐᵀ.

Relative to planar Lectures 2 and 4, N=G and B=D are renamings with unchanged meaning/sign. Their torque vector τ corresponds to our column τₐᵀ; row τₐ follows the original spatial lectures' dual convention.

## 2. Single-body inertia

In a body-fixed CoM frame cᵢ,

$$I^{c_i,i}=\begin{bmatrix}J^{c_i,i}&0\\0&m_iI_3\end{bmatrix},\qquad K_i=\frac12(T_i^{c_i,0})^TI^{c_i,i}T_i^{c_i,0}.$$

J is CoM rotational inertia in kg·m²; m is mass in kg. Principal axes diagonalize J. CoM origin and principal-axis orientation are separate choices: an arbitrarily oriented CoM frame can have off-diagonal J.

Spatial inertia differs from joint M(q). The former maps one body's 6D velocity to momentum; the latter maps generalized velocities to momenta in a constrained mechanism.

### Expressing inertia in another frame

Let \(T^k=A T^c\) with \(A=\operatorname{Ad}_{H_c^k}\). Energy invariance gives

$$\boxed{I^{k,i}=A^{-T}I^{c,i}A^{-1}.}$$

For equal frame orientations and CoM position r relative to the new origin:

$$I^{k,i}=\begin{bmatrix}J_c-m\widetilde r^{\,2}&m\widetilde r\\-m\widetilde r&mI_3\end{bmatrix}.$$

The upper-left block is the parallel-axis theorem, since −r̃²=‖r‖²I−rrᵀ. Cross blocks couple linear and angular momentum when the origin is not at the CoM.

## 3. Momentum, expression frame, and differentiation

$$\boxed{(P^{k,i})^T=I^{k,i}T_i^{k,0}.}$$

Momentum is dual to twist and transforms like a wrench. Changing only expression frame gives

$$P^{k,i}=P^{c,i}\operatorname{Ad}_{H_k^c}.$$

In an inertial frame with fixed origin:

$$\dot P^{0,i}=W_i^0.$$

In a rotating body frame, the ordinary derivative of momentum components is not simply the applied wrench: the basis also changes. Gyroscopic terms arise from this fact.

## 4. Small ad and Newton–Euler dynamics

For T=(w;v), define

$$\operatorname{ad}_T=\begin{bmatrix}\widetilde w&0\\\widetilde v&\widetilde w\end{bmatrix},\qquad\operatorname{ad}_T T=0.$$

Large Ad changes expression frames; small ad describes algebraic interaction of instantaneous motions. Both are 6×6 but have different origins.

Let Ψᵢ be body-fixed, I=Iⁱ˒ⁱ constant, and T=Tᵢⁱ˒⁰. Then

$$\boxed{\dot P=P\operatorname{ad}_T+W,}$$

or in column form,

$$\boxed{I\dot T-\operatorname{ad}_T^TIT=W^T.}$$

Check the sign using a CoM frame. For I=diag(J,mI),

$$J\dot w+w\times(Jw)=\tau^T,$$

$$m\dot v+m(w\times v)=F^T.$$

Here v uses rotating CoM coordinates. In an inertial frame the CoM equation is ma=F. Constant body-frame J does not justify omitting w×Jw.

### Energy and zero gyroscopic power

$$\dot K=T^TI\dot T=T^TW^T+T^T\operatorname{ad}_T^TIT=W T,$$

because \(T^T\operatorname{ad}_T^T=(\operatorname{ad}_TT)^T=0\). Gyroscopic coupling changes motion direction without generating or dissipating energy. Zero power does not mean the term can be omitted.

## 5. Free rotation and the intermediate axis

For J=diag(Jₓ,Jᵧ,J_z) with zero external moment:

$$\begin{aligned}J_x\dot w_x&=(J_y-J_z)w_yw_z,\\J_y\dot w_y&=(J_z-J_x)w_zw_x,\\J_z\dot w_z&=(J_x-J_y)w_xw_y.\end{aligned}$$

For Jₓ<Jᵧ<J_z, rotation about the intermediate principal axis is unstable to small transverse perturbations. Linearizing at w=(0,Ω,0) gives

$$\ddot w_x=\Omega^2\frac{(J_y-J_z)(J_x-J_y)}{J_xJ_z}w_x.$$

Both numerator factors are negative, so the coefficient is positive and perturbations grow. Kinetic energy and inertial angular momentum remain constant. This is intermediate-axis instability, also called the Dzhanibekov effect.

`rigid_body.py` uses J=(0.02;0.04;0.055) kg·m², dominant speed 5 rad/s, and small transverse components. RK4 integrates w; an exponential midpoint step integrates R at second order. Check inertial momentum drift separately from energy, even when w is very accurate. The model describes **free rotation about the CoM**, not translational falling.

## 6. Open-chain kinetic energy

For each link, use its geometric Jacobian Jᵢ in Ψ₀:

$$T_i^{0,0}=J_i(q)\dot q.$$

Only columns through joint i are nonzero. Later joints do not move earlier links. Express velocity in the body-fixed CoM frame:

$$T_i^{c_i,0}=\operatorname{Ad}_{H_0^{c_i}}J_i\dot q=:J_{b,i}\dot q.$$

Then

$$K=\sum_iK_i=\frac12\dot q^T\left(\sum_iJ_{b,i}^TI^{c_i,i}J_{b,i}\right)\dot q,$$

$$\boxed{M(q)=\sum_iJ_i^T\operatorname{Ad}_{H_0^{c_i}}^TI^{c_i,i}\operatorname{Ad}_{H_0^{c_i}}J_i.}$$

Dimensions: Jᵢ is 6×n, I is 6×6, M is n×n. Independent coordinates and physical nonzero inertias give symmetric positive-definite M. An entry Mᵢⱼ is not a link mass; it contributes to generalized-velocity energy.

## 7. Potential energy and gravity

Axis z₀ points upward and g₀=(0,0,−g). With CoM position p_cᵢ⁰,

$$P(q)=\sum_i m_ig\,e_z^Tp_{c_i}^0(q),\qquad\boxed{N(q)=\frac{\partial P}{\partial q}.}$$

N appears on the **left**. Gravity itself produces −N, so compensation adds +N to the motor command. Selecting (0,1,0) with vertical z would incorrectly pick y.

## 8. Euler–Lagrange derivation

With L=K−P:

$$\frac{d}{dt}\frac{\partial L}{\partial\dot q_i}-\frac{\partial L}{\partial q_i}=Q_i.$$

Use K=½∑Mⱼₖq̇ⱼq̇ₖ. Differentiation in q̇ᵢ gives ∑Mᵢⱼq̇ⱼ; its time derivative yields inertial acceleration and M derivatives. Grouping terms gives

$$\sum_jM_{ij}\ddot q_j+\sum_{j,k}\Gamma_{ijk}\dot q_j\dot q_k+\frac{\partial P}{\partial q_i}=Q_i,$$

$$\Gamma_{ijk}=\frac12\left(\frac{\partial M_{ij}}{\partial q_k}+\frac{\partial M_{ik}}{\partial q_j}-\frac{\partial M_{jk}}{\partial q_i}\right).$$

Define \(C_{ij}=\sum_k\Gamma_{ijk}\dot q_k\), so c=Cq̇. C is a matrix; c a vector. Prescribing Cq̇ alone does not uniquely determine C; we use the standard Christoffel construction.

## 9. Full dynamics and energy check

External row W₄⁰ is the **environment-on-robot** wrench about Ψ₀'s origin. With joint viscous damping B:

$$\boxed{M(q)\ddot q+C(q,\dot q)\dot q+N(q)+B\dot q=\tau_a^T+J_s^T(W_4^0)^T.}$$

The **robot-on-environment** force has opposite sign. Check this convention when comparing Chapter 7 of the source.

For the chosen C, Ṁ−2C is skew-symmetric. Therefore

$$\boxed{\frac{d}{dt}(K+P)=\tau_a\dot q+W_4^0T_4^{0,0}-\dot q^TB\dot q.}$$

This checks force signs, gravity, Jacobians, and loads. Numerical energy residual also depends on integration and power quadrature; do not expect exact machine-zero conservation.

## 10. Forward and inverse dynamics

**Inverse dynamics:** given q,q̇,q̈ and external load, find required actuator torque:

$$\tau_a^T=M\ddot q+C\dot q+N+B\dot q-J_s^T(W_4^0)^T.$$

**Forward dynamics:** given state and torques, find acceleration:

$$M\ddot q=\tau_a^T+J_s^T(W_4^0)^T-C\dot q-N-B\dot q.$$

Solve a linear system rather than forming M⁻¹. Then numerically integrate q,q̇. Forward dynamics alone is not an integrator.

Recursive Newton–Euler computes inverse dynamics without symbolically expanding M: forward velocity/acceleration propagation, backward wrench/torque propagation. This scales better for long chains. Explicit M and derivatives make our teaching derivation transparent; MuJoCo independently implements mechanics.

## 11. The specific 3R model

Geometry matches Lecture 5: home axes z,x,x, vertical post along z, two links along +y. Lengths (0.4;0.35;0.25) m, masses (2;1.5;1) kg, CoMs at midlinks. Inertias are uniform cylinders of radius r=0.025 m:

$$J_{\parallel}=\frac12mr^2,\qquad J_{\perp}=\frac{m(3r^2+\ell^2)}{12}.$$

Cylinder axes are z for link 1 and body y for links 2–3. XML inertias are explicit; visual capsules do not determine mass. B=0.06I N·m·s/rad; motors ±20 N·m; contacts disabled.

Let r₂=ℓ₂/2, r₃=ℓ₃/2, α=q₂+q₃, ρ=ℓ₂cos q₂+r₃cos α. Energy yields

$$M=\begin{bmatrix}\mu(q_2,q_3)&0&0\\0&a+2\beta\cos q_3&d+\beta\cos q_3\\0&d+\beta\cos q_3&d\end{bmatrix},$$

$$d=J_{3x}+m_3r_3^2,\quad\beta=m_3\ell_2r_3,\quad a=J_{2x}+m_2r_2^2+d+m_3\ell_2^2,$$

$$\begin{aligned}\mu={}&J_{1z}+m_2r_2^2\cos^2q_2+J_{2y}\sin^2q_2+J_{2z}\cos^2q_2\\&+m_3\rho^2+J_{3y}\sin^2\alpha+J_{3z}\cos^2\alpha.\end{aligned}$$

Zero M₁₂,M₁₃ do not imply uncoupled dynamics: μ depends on q₂,q₃, creating Coriolis coupling to joint 1. Vertical-axis symmetry makes M independent of q₁ and gives N₁=0:

$$N=\begin{bmatrix}0\\g(m_2r_2+m_3\ell_2)\cos q_2+m_3gr_3\cos\alpha\\m_3gr_3\cos\alpha\end{bmatrix}.$$

`model.py` evaluates M/derivatives analytically and C through Christoffel symbols. Independent `mass_from_energy` assembles M from CoM Jacobians and spatial inertias rather than repeating the simplified expression.

## 12. Chapter 7 control: from model to behavior

### Gravity compensation

τₐᵀ=N(q) preserves a stationary initial state in an ideal unloaded model. It does not track arbitrary trajectories and creates no position spring restoring the original pose after disturbance.

### Feedback linearization

Let \(e=\bar q-q\) as in source Chapter 7. Choose

$$u=\ddot{\bar q}+K_v\dot e+K_pe,$$

$$\tau_a^T=\hat M(q)u+\hat C(q,\dot q)\dot q+\hat N(q)+\hat B\dot q.$$

With exact model, no external load, and no saturation, q̈=u and ë+Kᵥė+Kₚe=0. Here Kp=400I s⁻², Kv=40I s⁻¹ (ω=20 rad/s, ζ=1). Model error leaves residual force; `--model-mass-scale .8` reduces only **nominal controller masses/inertias**, retaining physical XML.

### Force interface and sign

For desired **robot-on-environment** wrench W_des in quasistatic contact, command τₐᵀ=N+JᵀW_desᵀ. Environment reaction is opposite. Achieving force still requires contact and appropriate model/feedback. Force control does not require all joint rates to vanish; one direction may be constrained while others move.

### Spatial virtual spring

Consider spatial elasticity defined by a potential without cross-coupling between rotation and translation. Target (R_d,p_d) is constant and reachable. Let R=R₄⁰, p=o₄⁰:

$$U_s=\frac{k_p}{2}\|p-p_d\|^2+\frac{k_o}{2}\operatorname{tr}(I-R_d^TR).$$

$$e_R=\frac12\operatorname{vee}(R_d^TR-R^TR_d),\qquad F_s=-k_p(p-p_d)-d_p\dot p,$$

$$\tau_{\rm tip}^0=-k_oR e_R-d_o w_4^{0,0}.$$

e_R uses current body coordinates; R maps its torque to Ψ₀. Similar-looking formulas without frame identification can mean different things.

Use Jω for angular velocity and Jp for **tip-point velocity**:

$$\boxed{\tau_a^T=N+J_p^TF_s+J_\omega^T\tau_{\rm tip}^0.}$$

The equivalent spatial wrench row about **Ψ₀'s origin** is

$$W_s^0=\begin{bmatrix}(\tau_{\rm tip}^0+p\times F_s)^T&F_s^T\end{bmatrix}.$$

Then Jsᵀ(W_s⁰)ᵀ gives the same joint torque. Supplying a tip-relative moment without p×F produces a different physical wrench.

With exact N compensation, fixed target, and no saturation:

$$\frac{d}{dt}(K+U_s)=F_{\rm ext}^T\dot p-d_p\|\dot p\|^2-d_o\|w\|^2-\dot q^TB\dot q.$$

Active gravity compensation removes gravitational potential from this storage function. This is a passivity argument, not global asymptotic convergence to arbitrary orientation: SO(3) potential has extra critical orientations at 180°. Three drives cannot independently set six pose coordinates. Our target is the reachable initial joint pose.

## 13. Experiments

- `rigid_body.py`: free rotation about minimum, intermediate, or maximum axes; plots w, energy, and inertial angular momentum.
- `simulate.py --method free`: gravity/friction motion without motors. The visual plane does not stop links because collisions are disabled.
- `--method gravity`: ideal initial-pose holding through N compensation.
- `--method computed-torque`: smooth joint tracking with analytical q̄,q̄̇,q̄̈.
- `--method computed-torque --model-mass-scale .8`: same physical robot with an inaccurate controller model.
- `--method impedance --force`: reachable-pose spatial spring/damper response to a +x pulse, 3 N between 3–5 s, followed by return.

Known force acts at tip through `mj_applyFT`; this is neither a sensor model nor wall contact. Reset `qfrc_applied` each step. Control is 500 Hz; MuJoCo `implicitfast` uses 2 ms steps. Viewer data is locked. Direct and module execution are supported.

Tip error to reference means **different things** for free motion, holding, tracking, and compliance. The combined figure describes experiments, not controller rankings. For computed torque inspect joint error too; for compliance inspect displacement, return, energy, and saturation.

Slides embed real MuJoCo recordings. Separate inertia, gyroscopic, and pose controls use labeled analytical models.

### Verified default results

MuJoCo 3.10.0, 8 s, 2 ms, one physical XML and the same computed-torque reference:

| Nominal controller model | RMS tip, mm | Maximum, mm | Saturation |
|---|---:|---:|---:|
| Exact | 0.00774 | 0.01186 | 0% |
| Masses/inertias ×0.8 | 6.10717 | 7.37181 | 0% |

The spatial spring pulse produces about 22.48 mm maximum displacement without saturation. This is intended compliance, not exact tracking of the same moving target.

For separate free rotation (10 s, 2 ms), maximum K drift is about 2.1·10⁻¹² J and L⁰ residual about 8.7·10⁻⁷ N·m·s. Their difference is consistent with different w/R integration orders. These characterize numerical experiments, not hardware accuracy.

[Free rotation](spatial_dynamics/reference_results/rigid.png) · [spring and pulse](spatial_dynamics/reference_results/impedance.png) · [inaccurate model](spatial_dynamics/reference_results/mismatch.png).

## 14. Questions and exercises

1. Derive inertia transformation from kinetic-energy invariance. Why not simply AᵀIA when T_new=A T_old?
2. Show ad_T T=0 and derive zero gyroscopic power.
3. Why does constant body J not eliminate w×Jw?
4. Compute N₁ for the vertical first axis. Why zero despite nonzero robot mass?
5. Check dimensions of JᵢᵀAdᵀIAdJᵢ and the gravity-compensation sign.
6. Locate three independent M implementations in `verify.py`: analytical, energy-based, MuJoCo. Why is testing only inverse(forward()) weaker?
7. Compare model scales 1 and 0.8 without changing physical XML. Which terms become inaccurate?
8. Derive world-origin moment for a tip force and verify JsᵀWᵀ=JpᵀF+Jωᵀτ_tip.
9. Predict gravity-compensation response after a pulse. What restores initial pose? **Answer:** N alone creates no restoring position action; friction only slows motion.
10. Halve free-rotation step. How do energy/inertial-momentum residuals change with RK4 for w but second-order R integration?

## 15. Checking formulas and conventions

| Original PDF | Convention here |
|---|---|
| Chapter 6: constant J used for τ=Jẇ | Include w×Jw in body coordinates; inertial J usually varies |
| Chapter 6: momentum/inertia expression indices sometimes differ | I,T,P always use one expression frame |
| Chapter 6: C called a vector yet multiplied by q̇ | C is a matrix; c=Cq̇ is a vector |
| Chapter 6: y selected despite vertical z | Use e_z=(0,0,1) |
| Chapter 6: N versus G | N=G=∇P; fix N in main notation |
| Chapter 7: external-force/compensation signs ambiguous | Distinguish environment-on-robot from robot-on-environment |
| Chapter 7: force control tied to q̇=0 | Contact direction can be constrained while other motion continues |
| Chapter 7: general co-stiffness spring | Use a derived energy-based special case, not all original coupling terms |

## 16. References

- Kevin M. Lynch, Frank C. Park. **Modern Robotics: Mechanics, Planning, and Control**, Cambridge University Press, 2017. [Official site](https://modernrobotics.northwestern.edu/).
- [Chapter 8: Dynamics of Open Chains](https://modernrobotics.northwestern.edu/chapters/chapter8/): §8.1 Lagrange/M, §8.2 rigid-body dynamics, §8.3 Newton–Euler inverse dynamics, §8.5 forward dynamics.
- [§8.3: Newton–Euler Inverse Dynamics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-3-newton-euler-inverse-dynamics/), [§8.5: Forward Dynamics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-5-forward-dynamics-of-open-chains/).
- [§11.4, Part 3: computed torque](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-4-motion-control-with-torque-or-force-inputs-part-3-of-3/); §§11.5–11.6 force/hybrid control.
- Original [Chapter 6](../../SRS/lecture_6/lecture_6.pdf) and [Chapter 7](../../SRS/lecture_7/lecture_7.pdf). See their Stramigioli–Duindam and Fasse–Broenink references for general spatial co-stiffness; our example uses a simpler potential.
- MuJoCo: [computation/integrators](https://mujoco.readthedocs.io/en/stable/computation/), [mj_inverse](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-inverse), [mj_applyFT](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-applyft), [passive viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer).

In Modern Robotics, configuration T corresponds to our H, velocity V to our twist T, spatial inertia 𝒢 to our I, and its column wrench to Wᵀ. Apply this mapping before copying symbols or transposes.

