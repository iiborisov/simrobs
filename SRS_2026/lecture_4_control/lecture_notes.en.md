[Русский](lecture_notes.md) | **English**

# Lecture 4. 2R Control: Feedback Linearization, Impedance, and Admittance

**Duration: 90 minutes.** Students should derive computed-torque control, explain two compliant-control interfaces, and test them on one MuJoCo model.

[Presentation](presentation/index.en.html) · [Examples and commands](control_2r/README.en.md) · [Earlier 2R dynamics lecture](../lecture_2_2R_robot/lecture_notes.en.md).

Based on [SRS/lecture_3/lecture_3.pdf](../../SRS/lecture_3/lecture_3.pdf), especially §§3.3–3.4: gravity compensation, Cartesian virtual springs/dampers, and environmental interaction. This version adds link rotational inertias and specifies three distinct laws. Feedback linearization and admittance extend the example; coefficients are not copied literally from the PDF.

## Session plan

| Time | Topic |
|---|---|
| 0–13 min | Problem, coordinates, kinematics, dynamics |
| 13–38 min | From PD to computed torque: derivation and limits |
| 38–53 min | Force, power, Cartesian impedance |
| 53–65 min | Admittance and inner tracking loop |
| 65–83 min | MuJoCo: figure eight, force pulse, tuning |
| 83–90 min | Questions and discussion |

## 1. What do we control?

Forward dynamics asks how a robot moves under given torques. Control asks which torques, based on a reference and measured state, produce desired behavior. Inverse dynamics helps calculate torques but needs feedback to correct deviations.

Motion is in the vertical **xz** plane: x right, z up, gravity down. q₁ runs from +x toward +z; q₂ is relative. Tip position p=(x,z) is **shoulder-relative**; the MuJoCo shoulder is 0.3 m above world zero. XML hinge axis `(0,-1,0)` matches positive angles.

Both links are 0.4 m long and 2 kg, with CoM distance 0.2 m and transverse CoM inertia I=mℓ²/12. Joint damping is b=0.05 N·m·s/rad; g=9.81 m/s². Motors are limited to ±35 N·m. Contacts are disabled: interaction uses a known external force, not wall collision.

## 2. Kinematic interface

$$p(q)=\begin{bmatrix}\ell_1\cos q_1+\ell_2\cos(q_1+q_2)\\\ell_1\sin q_1+\ell_2\sin(q_1+q_2)\end{bmatrix}.$$

$$\dot p=J(q)\dot q,\qquad \ddot p=J\ddot q+\dot J\dot q,$$

$$J=\begin{bmatrix}-\ell_1\sin q_1-\ell_2\sin(q_1+q_2)&-\ell_2\sin(q_1+q_2)\\\ell_1\cos q_1+\ell_2\cos(q_1+q_2)&\ell_2\cos(q_1+q_2)\end{bmatrix}.$$

Function `joint_reference` selects the continuous positive-sin(q₂) IK branch, then solves

$$J(q_r)\dot q_r=\dot p_r,\qquad J(q_r)\ddot q_r=\ddot p_r-\dot J(q_r,\dot q_r)\dot q_r.$$

Use J and J̇ **at the reference state** when differentiating the reference. J⁻¹ is meaningful away from det J=ℓ₁ℓ₂sin q₂=0. Code uses `np.linalg.solve` instead of an explicit inverse. Unreachable targets raise an exception rather than being silently clipped.

## 3. Dynamics and force signs

$$\boxed{M(q)\ddot q+c(q,\dot q)+G(q)+B\dot q=\tau+J(q)^T F_{\rm ext}.}$$

F_ext is force **from environment to robot**, in N, positive along +x,+z. τ is motor torque in N·m. c is a centrifugal/Coriolis vector, not matrix C; B=bI. Gravity remains in the plant and is compensated by positive +G in control.

Let β=m₂ℓ₁r₂, d=I₂+m₂r₂², a=I₁+I₂+m₁r₁²+m₂(ℓ₁²+r₂²). Then

$$M=\begin{bmatrix}a+2\beta\cos q_2&d+\beta\cos q_2\\d+\beta\cos q_2&d\end{bmatrix},\quad c=\beta\sin q_2\begin{bmatrix}-2\dot q_1\dot q_2-\dot q_2^2\\\dot q_1^2\end{bmatrix}.$$

$$G=\begin{bmatrix}(m_1r_1+m_2\ell_1)g\cos q_1+m_2gr_2\cos(q_1+q_2)\\m_2gr_2\cos(q_1+q_2)\end{bmatrix}.$$

M₂₂ includes I₂. Point masses have I₂=0; the uniform rod has I₂≠0. Mixing these models yields incorrect compensation.

## 4. From PD to feedback linearization

Simple PD with gravity compensation:

$$\tau=K_p^{\tau}(q_d-q)+K_d^{\tau}(\dot q_d-\dot q)+G(q).$$

It can hold a static pose; during motion, errors must generate inertial and Coriolis torques. Joint coupling remains. PD+G and computed torque are different laws.

Define e=q_d−q and virtual acceleration

$$v=\ddot q_d+K_d\dot e+K_pe,$$

then obtain physical torque by inverse dynamics:

$$\boxed{\tau=M(q)v+c(q,\dot q)+G(q)+B\dot q.}$$

Evaluate M at **current** q and c at current q,q̇. Reference acceleration q̈_d comes from the command. With exact model, no external force, and no saturation, substitution gives

$$M\ddot q=Mv\Rightarrow\ddot q=v\Rightarrow\boxed{\ddot e+K_d\dot e+K_pe=0.}$$

This is **feedback linearization**: a nonlinear torque law produces linear closed-loop error dynamics. The physical robot remains nonlinear; the claim concerns the closed loop under these assumptions. Fully actuated 2R has positive-definite M throughout joint space. J singularities arise when converting Cartesian references to joint references.

For positive diagonal gains, each error obeys ëᵢ+2ζᵢωᵢėᵢ+ωᵢ²eᵢ=0. The code uses ω=35 rad/s, ζ=1: Kp=1225I s⁻², Kd=70I s⁻¹, unlike direct torque-PD units. For general symmetric positive-definite Kp,Kd, stability follows, for example, from ½ėᵀė+½eᵀKpe.

### Model error, saturation, and external force

With an estimated model, actual commanded torque is τ=M̂v+ĉ+Ĝ+B̂q̇. Then

$$\ddot e+K_d\dot e+K_pe=-M^{-1}r,$$

$$r=(\hat M-M)v+(\hat c-c)+(\hat G-G)+(\hat B-B)\dot q+J^TF_{\rm ext}+\Delta\tau_{\rm sat}.$$

Here Δτ_sat=τ_applied−τ_command. Exact compensation cannot be promised under saturation, unknown payloads, delay, or mass error. Large Kd amplifies measured velocity noise. Large gains increase torque/update-rate demands; continuous-time stability does not prove discrete implementation stability.

## 5. Force, Jacobian, and energy port

Virtual work or power equivalence gives

$$\tau_{\rm ext}=J^TF_{\rm ext},\qquad\dot q^T\tau_{\rm ext}=\dot p^TF_{\rm ext}.$$

Jᵀ maps force to torque; nonsingular J⁻¹ maps Cartesian to joint velocity. These are different operations. External force already enters the dynamics on the right; do not add it again to control as a simulator disturbance.

## 6. Impedance: motion → force

For stationary p_d, choose a Cartesian virtual spring and damper:

$$F_c=K_x(p_d-p)+D_x(\dot p_d-\dot p),\qquad\boxed{\tau=G(q)+J^TF_c.}$$

This is **impedance control without arbitrary inertia shaping**. Here Kx=120I N/m, Dx=22I N·s/m. Output is torque; position may yield under load. Physical inertia, Coriolis terms, and joint friction remain. For moving targets this is compliant tracking, not exact trajectory reproduction.

At rest with full-rank J:

$$0=J^T[K_x(p_d-p)+F_{\rm ext}]\Rightarrow p-p_d=K_x^{-1}F_{\rm ext}.$$

Force +4 N with Kx=120 N/m gives a **static** +33.3 mm offset, not necessarily the peak under a short pulse.

For constant p_d, mechanical and virtual-spring energy

$$H=\tfrac12\dot q^TM\dot q+\tfrac12(p-p_d)^TK_x(p-p_d)$$

satisfies

$$\dot H=F_{\rm ext}^T\dot p-\dot p^TD_x\dot p-\dot q^TB\dot q.$$

With exact G compensation and no saturation, the system is passive at (F_ext,ṗ): it produces no energy without supply. A moving reference adds another energy input. This does not automatically guarantee stability for every digital implementation or contact.

### Why this does not impose desired mass M_d

Away from singularities, physical effective tip inertia

$$\Lambda(q)=(JM^{-1}J^T)^{-1}$$

depends on configuration. Jᵀ(Kxe+Dxė)+G does not turn it into an arbitrary constant. Full impedance shaping requires dynamic compensation and correct measured-force treatment when shaping interaction inertia. This example deliberately sets virtual stiffness/damping while retaining physical inertia.

## 7. Admittance: force → motion

An outer admittance loop receives measured force and produces a reference offset:

$$\boxed{M_a\ddot\delta+D_a\dot\delta+K_a\delta=F_{\rm ext},}\qquad p_c=p_d+\delta.$$

Here Ma=1.5I kg, Da=22I N·s/m, Ka=120I N/m. Ma differs from joint M(q) in units and space. Filter state is (δ,δ̇), initially zero. Inner computed torque tracks p_c through IK,J,J̇, with ṗ_c and p̈_c supplied consistently. Position-only updates create unnecessary tracking error.

The filter uses **classical RK4**, step 2 ms. MuJoCo `implicitfast` integrates the robot. These are different solvers for different subsystems. The prescribed force is known continuously, enabling intermediate RK4 evaluations; a real sampled sensor requires a hold/interpolation model.

Even with an exact model, the inner loop does not automatically remove physical disturbance:

$$\ddot e_q+K_d\dot e_q+K_pe_q=-M^{-1}J^TF_{\rm ext},\qquad e_q=q_c-q.$$

Thus actual p only approximately equals p_c. Force is **not subtracted** from torque to perfectly cancel the disturbance. Slides show p_d,p_c,p separately. Real systems require attention to inner bandwidth, force noise/filtering, position/velocity limits, delay, and p_c reachability.

### Virtual-filter stability

For one axis, δ̈+(Da/Ma)δ̇+(Ka/Ma)δ=F/Ma. Positive Ma,Da,Ka place poles in the left half-plane:

$$\omega_a=\sqrt{K_a/M_a},\qquad\zeta_a=D_a/(2\sqrt{M_aK_a}).$$

Nominally ωa≈8.94 rad/s, ζa≈0.82. Lower Ma speeds response and tightens sampling demands; lower Ka increases static compliance; lower Da increases oscillations. At Ka=0 a constant force has no finite static offset, a regime excluded by this CLI.

## 8. Comparing interfaces

| Method | Input | Output | Compliance mechanism |
|---|---|---|---|
| Computed torque | q,q̇ and joint reference/derivatives | τ | Here the goal is suppressing motion error |
| Impedance | p,ṗ and nominal trajectory | τ through Jᵀ | Virtual spring/damper torque |
| Admittance | F_ext | p_c/derivatives, then inner torque | Virtual dynamics moves the target |

No method is universally best. Torque access naturally supports impedance; an existing position servo often suits outer admittance. These are engineering guides, not prohibitions on other architectures.

## 9. Reproducible MuJoCo experiments

See [README](control_2r/README.en.md). Direct scripts work from any directory; `python -m` works from the repository root. No manual PYTHONPATH addition is required.

**A. Tracking:** p_d=(0.4+0.12sinθ;0.2+0.10sin2θ), period 6 s, phase ramp 2 s. Without external force the admittance filter remains zero, making admittance identical to computed torque. This checks the architecture; it is not an independent third accuracy result.

**B. Interaction:** fixed p_d=(0.4;0.2) m and known force

$$F_x(t)=\begin{cases}4\sin^2\!\left(\pi(t-3)/2\right),&3<t<5,\\0,&\text{otherwise},\end{cases}\qquad F_z=0.$$

Apply it at tip through `mj_applyFT`. The same force is the admittance loop's idealized measurement. **No physical force sensor or noise model is included.** Reset `qfrc_applied` before every application to prevent accumulation. Contacts are disabled and force does not depend on position.

Each step: synchronize via `mj_forward` → read state → compute control → clip torque → apply force → log synchronized data → update filter → `mj_step`. Viewer mutations occur under its lock.

CSV stores actual p, nominal p_d, commanded p_c, force, clipped torques, and saturation flag. RMS to p_d measures nominal deviation, possibly desired under compliance. RMS to p_c measures inner tracking. Inspect saturation separately.

Verified MuJoCo 3.10.0 run, 0.002 s step, 10 s, no external force:

| Method | RMS to p_d, mm | Maximum, mm | Saturation |
|---|---:|---:|---:|
| Computed torque | 0,0175 | 0,0269 | 0% |
| Impedance | 7,0776 | 16,0707 | 0% |
| Admittance | 0,0175 | 0,0269 | 0% |

Under a 4 N holding-task pulse, maximum nominal deviations are 4.84 mm for computed torque, 31.74 mm for impedance, and 37.06 mm for admittance. Admittance RMS to p_c is about 1.31 mm. No saturation occurs; the robot returns after release.

Values describe this experiment/integrator version. Smaller steps change discretization error; tiny ideal-simulation error does not prove comparable hardware accuracy.

## 10. Classroom exercises

1. **Derivation, 4 min.** Substitute computed torque into disturbed dynamics. Check the error-equation sign: −M⁻¹JᵀF_ext for e=q_d−q.
2. **Units, 2 min.** Why cannot Kp=1225 replace Kx=120 directly? Answer: s⁻² for virtual acceleration versus N/m for Cartesian force.
3. **Tuning, 4 min.** Reduce stiffness from 120 to 60 N/m in interaction. Static displacement doubles, but a short-pulse peak also depends on inertia and damping.
4. **Damping, 4 min.** Compare Da=4,22,50 at Ma=1.5. Predict overshoot and return speed; repeat with the slide filter.
5. **Time step, 3 min.** Compare `--step .004`, `.002`, `.001` for computed torque. Record RMS and peak torque; keep gains fixed.
6. **Model, 3 min.** Change XML mass only, retaining controller analytics. How does this differ from changing both consistently? Restore XML afterward.
7. **Discussion.** Why can impedance have larger position error while correctly accomplishing interaction?

## 11. References and documentation

- Kevin M. Lynch, Frank C. Park, **Modern Robotics: Mechanics, Planning, and Control**, Cambridge University Press, 2017. [Official website/materials](https://modernrobotics.northwestern.edu/).
- Chapter 11, §11.4: torque control, PD/PID, computed torque. [Computed torque, Part 3](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-4-motion-control-with-torque-or-force-inputs-part-3-of-3/) is the main feedback-linearization reference here.
- Chapter 8: open-chain dynamics; [§8.3 inverse dynamics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-3-newton-euler-inverse-dynamics/), [§8.5 forward dynamics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-5-forward-dynamics-of-open-chains/).
- Chapter 5: Jacobians/statics and velocity-force mappings; §§11.5–11.6: force and hybrid motion-force control. These complement interaction, but **§11.6 is not named “impedance/admittance”**, and hybrid control is distinct.
- Original [§§3.3–3.4, SRS/lecture_3](../../SRS/lecture_3/lecture_3.pdf): virtual springs/dampers and 2R interaction.
- MuJoCo: [computation/integrators](https://mujoco.readthedocs.io/en/stable/computation/), [mj_applyFT](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-applyft), [Python/passive viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer). `mj_applyFT` adds equivalent generalized force to a supplied vector; it is not a sensor reading.

Read §11.4 and `controllers.py`, then compare `comparison.png` and CSV columns p_d,p_c,p. Slides and notes use the same signs.

