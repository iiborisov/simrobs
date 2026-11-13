[Русский](lecture_notes.md) | **English**

# Lecture 5. Rigid-Body Kinematics in Three-Dimensional Space

**90 minutes.** From point coordinates to SO(3), SE(3), twists, screw motion, and spatial 3R kinematics.

[Presentation](presentation/index.en.html) · [Python and MuJoCo](spatial_kinematics/README.en.md) · [Slide schedule](presentation/TEACHING_PLAN.en.md).

Main sources: [Chapter 4](../../SRS/lecture_4/lecture_4.pdf) and [Chapter 5](../../SRS/lecture_5/lecture_5.pdf) of the original lectures. Additional source: Lynch and Park, *Modern Robotics*, Chapters 3–5. Use **H** for configuration and **T** for a twist. Modern Robotics uses T for configuration; a notation correspondence table appears at the end.

## 1. Body, frame, and representation

A rigid body keeps distances between material points constant. Position and orientation together form its configuration. A free spatial body has six degrees of freedom: three translational and three rotational. Nine rotation-matrix entries are constrained by orthogonality, not nine independent angles.

A physical point p differs from its coordinate column pⁱ. The same point can be expressed in different frames Ψᵢ. Changing coordinates does not physically move the point.

| Symbol | Meaning |
|---|---|
| Ψᵢ | Right-handed orthonormal frame |
| pⁱ ∈ ℝ³ | Coordinates of p in Ψᵢ |
| oᵢʲ ∈ ℝ³ | Origin of Ψᵢ expressed in Ψⱼ |
| Rᵢʲ | Vector coordinate map from Ψᵢ to Ψⱼ |
| Hᵢʲ | Homogeneous point coordinate map from Ψᵢ to Ψⱼ |
| wᵢᵏ˒ʲ | Angular velocity of body i relative to j, expressed in Ψₖ |
| Tᵢᵏ˒ʲ=(wᵢᵏ˒ʲ;vᵢᵏ˒ʲ) | Twist: **angular first, linear second**, 6×1 column |
| Wᵢᵏ=(τᵢᵏ Fᵢᵏ) | Wrench: moment and force, **1×6 row** |
| ã | Skew-symmetric matrix satisfying ãb=a×b |

Full indices are \(R_i^j,H_i^j,T_i^{k,j},W_i^k\). Code `H_i_j` reads in the same order, i to j. T means **twist**, not configuration. Physical power is \(\mathcal P\) to distinguish it from homogeneous point P.

Lectures 1–4 use T for kinetic energy. From spatial kinematics onward, T means twist; Lecture 6 uses K for kinetic energy. Configuration is H in Lectures 2,5,6, with 3×3 or 4×4 size for planar or spatial motion.

## 2. Rotation matrices: interpreting columns

For a vector in frames sharing an origin,

$$p^j=R_i^j p^i,\qquad R_i^j=\begin{bmatrix}(\hat x_i)^j&(\hat y_i)^j&(\hat z_i)^j\end{bmatrix}.$$

Columns are source-frame axes Ψᵢ expressed in destination Ψⱼ. Points in frames with different origins additionally require translation.

$$SO(3)=\{R\in\mathbb R^{3\times3}:R^TR=I,\ \det R=1\}.$$

R preserves lengths and angles; det R=+1 excludes reflection. Identity is I, and inverse map is \(R_j^i=(R_i^j)^T\). Multiplication is associative but generally **not commutative**.

Let Ψ₂ rotate +θ about z₁ relative to Ψ₁. Then

$$R_2^1=\begin{bmatrix}\cos\theta&-\sin\theta&0\\\sin\theta&\cos\theta&0\\0&0&1\end{bmatrix},\quad R_1^2=(R_2^1)^T.$$

The negative angle in \(R_1^2\) is the first-to-second coordinate change in the original planar example. A +θ matrix has **opposite index direction**, not contradictory physics. Active point rotation and passive coordinate change may use similar matrices; identify the operation and frames first.

### Chain rule

$$p^k=R_j^k(R_i^j p^i),\qquad\boxed{R_i^k=R_j^kR_i^j.}$$

The transform nearest the vector acts first. For example, \(R_x(\pi/2)R_z(\pi/2)e_x=e_z\), whereas \(R_z(\pi/2)R_x(\pi/2)e_x=e_y\).

## 3. Skew matrices and angular velocity

$$\widetilde a=\begin{bmatrix}0&-a_z&a_y\\a_z&0&-a_x\\-a_y&a_x&0\end{bmatrix},\qquad\widetilde a b=a\times b.$$

For smooth R(t), differentiate RRᵀ=I:

$$\dot R R^T+R\dot R^T=0\quad\Rightarrow\quad(\dot RR^T)^T=-\dot RR^T.$$

Thus \(\dot RR^T\) is not a rotation matrix but an element of \(\mathfrak{so}(3)\), the skew-symmetric matrices. Geometrically it represents angular velocity.

$$\widetilde w_i^{j,j}=\dot R_i^jR_j^i,\qquad\widetilde w_i^{i,j}=R_j^i\dot R_i^j.$$

Both describe the same relative rotation in different expression frames:

$$w_i^{j,j}=R_i^jw_i^{i,j},\qquad\dot R_i^j=\widetilde w_i^{j,j}R_i^j=R_i^j\widetilde w_i^{i,j}.$$

Angular velocity generally differs from the vector of Euler-angle derivatives. A configuration-dependent map relates them and may become singular. We use R and twists without introducing an Euler-angle coordinate convention.

For a point fixed in Ψᵢ and coincident frame origins:

$$\dot p^j=\dot R_i^jp^i=\widetilde w_i^{j,j}p^j=w_i^{j,j}\times p^j.$$

If p moves relative to the body, add \(R_i^j\dot p^i\).

## 4. Rotation exponential and Rodrigues' formula

Constant spatial angular velocity gives ODE \(\dot R=\widetilde wR\) with solution

$$R(t)=\exp(\widetilde wt)R(0).$$

Constant body-expressed velocity gives the opposite order: \(R(t)=R(0)\exp(\widetilde w_b t)\). For varying velocity, an exponential of its integral is not generally valid because generators at different times may not commute.

Let \(\hat w\) be a unit axis and θ an angle. Then

$$\boxed{R=\exp(\widetilde{\hat w}\theta)=I+\sin\theta\,\widetilde{\hat w}+(1-\cos\theta)\widetilde{\hat w}^{\,2}.}$$

A unit axis differs from integrated rotation vector \(\phi=\hat w\theta\). For nonunit φ, the same formula is

$$R=I+\frac{\sin\theta}{\theta}\widetilde\phi+\frac{1-\cos\theta}{\theta^2}\widetilde\phi^2,\quad\theta=\|\phi\|.$$

As θ→0, coefficients approach 1 and 1/2. The code uses series to avoid division by zero and cancellation. Matrix log gives local exponential coordinates, not a globally unique representation. At θ=π, axis sign is ambiguous; at zero rotation, the axis is undefined. These are not physical orientation singularities.

### Numerical experiment

Additive Euler step \(R_{n+1}=R_n+h\widetilde wR_n\) generally violates RᵀR=I. Structural step \(R_{n+1}=\exp(h\widetilde w)R_n\) preserves SO(3) to rounding accuracy. It is exact for constant w; with varying w held at the start of each step, it is generally first order. Geometry preservation and accuracy order are different properties.

## 5. Body configuration: SE(3)

Frame origins may differ:

$$p^j=R_i^jp^i+o_i^j.$$

Introduce \(P^i=((p^i)^T,1)^T\):

$$\boxed{P^j=H_i^jP^i,\qquad H_i^j=\begin{bmatrix}R_i^j&o_i^j\\0&1\end{bmatrix}.}$$

$$SE(3)=\left\{\begin{bmatrix}R&o\\0&1\end{bmatrix}:R\in SO(3),\ o\in\mathbb R^3\right\}.$$

A free vector has fourth coordinate 0, so translation does not affect it. The difference of homogeneous points also has final coordinate 0.

### Inversion and composition

$$\boxed{H_j^i=(H_i^j)^{-1}=\begin{bmatrix}(R_i^j)^T&-(R_i^j)^To_i^j\\0&1\end{bmatrix}.}$$

For H, transposition **is not inversion**, even at R=I when translation is nonzero.

$$\boxed{H_i^k=H_j^kH_i^j},\qquad o_i^k=R_j^ko_i^j+o_j^k.$$

Express translations in one frame before adding them. Simply adding oᵢʲ+oⱼᵏ is generally incorrect.

Numerical example: \(R_i^j=R_z(\pi/2)\), \(o_i^j=(0.3,-0.2,0.4)^T\), \(p^i=(0.2,0.1,0)^T\) gives \(p^j=(0.2,0,0.4)^T\). The script checks the inverse map.

## 6. Twist as configuration velocity

For H=Hᵢʲ(t), define

$$\boxed{\widetilde T_i^{j,j}=\dot H_i^jH_j^i,\qquad\widetilde T_i^{i,j}=H_j^i\dot H_i^j.}$$

These belong to algebra \(\mathfrak{se}(3)\):

$$\widetilde T=\begin{bmatrix}\widetilde w&v\\0&0\end{bmatrix},\qquad T=\begin{bmatrix}w\\v\end{bmatrix}\in\mathbb R^6.$$

Unlike H, a twist matrix has a zero final row. Tilde denotes matrix representation: 3×3 for rotation vectors, 4×4 for twists. Code `hat`/`vee` converts twists; `skew`/`unskew` converts 3-vectors.

### Meaning of linear component v

Write H=(R,o). Direct multiplication gives

$$v_i^{j,j}=\dot o_i^j-\widetilde w_i^{j,j}o_i^j,\qquad v_i^{i,j}=R_j^i\dot o_i^j.$$

**Spatial linear component v usually differs from body-origin velocity ȯ.** It is the velocity field evaluated at the expression-frame origin Ψⱼ. For any body point in Ψⱼ,

$$\boxed{\dot p^j=w_i^{j,j}\times p^j+v_i^{j,j}.}$$

Extend this field throughout space: no actual material point need occupy Ψⱼ's origin. A twist therefore describes the whole body, not one selected point.

At the body origin, \(\dot o=\widetilde wo+v\). Rotation with w=(0,0,ω) around z through r=(a,0,0) has \(v=r\times w=(0,-a\omega,0)\). Points on the axis are stationary, yet v is nonzero because it evaluates the field at another origin.

## 7. Adjoint: changing the twist expression frame

Keep motion of i relative to j fixed and change **only the expression frame** from Ψⱼ to Ψₖ:

$$\widetilde T_i^{k,j}=H_j^k\widetilde T_i^{j,j}H_k^j.$$

$$\boxed{T_i^{k,j}=\operatorname{Ad}_{H_j^k}T_i^{j,j},\qquad\operatorname{Ad}_{H}=\begin{bmatrix}R&0\\\widetilde oR&R\end{bmatrix}.}$$

Order (w;v) determines block placement. Libraries using (v;w) have a different matrix. For one physical velocity, \(T_i^{j,j}=\operatorname{Ad}_{H_i^j}T_i^{i,j}\).

This does not change the reference body: replacing the last index j with another moving body requires relative-motion accounting. Sum link twists only after expressing them in one frame:

$$T_n^{0,0}=T_1^{0,0}+T_2^{0,1}+\cdots+T_n^{0,n-1}.$$

## 8. Wrenches and power invariance

The original lectures represent spatial force as a **row**:

$$W_i^k=\begin{bmatrix}\tau_i^k&F_i^k\end{bmatrix},\qquad\mathcal P=W_i^kT_i^{k,j}.$$

τ is moment about Ψₖ's origin, not merely motor torque. Angular velocity is in rad/s, linear velocity in m/s, torque in N·m, force in N. Power in W is frame-invariant.

If \(T^{k}=\operatorname{Ad}_{H_j^k}T^{j}\), equality \(W^kT^k=W^jT^j\) gives

$$\boxed{W_i^k=W_i^j\operatorname{Ad}_{H_k^j}.}$$

For a **column** wrench this uses an inverse transpose; do not copy that rule blindly to rows. NumPy uses a 1D array, but `W @ Ad` follows the row convention.

Moving the origin changes moment: a force applied at r creates \(\tau=r\times F\). Zero moment depends on origin; a pure couple F=0 acquires no force when the origin changes.

## 9. Screw interpretation

For w≠0, any twist decomposes into rotation about an axis and translation along it:

$$v=r\times w+\lambda w,\qquad\lambda=\frac{w^Tv}{\|w\|^2},\qquad r=\frac{w\times v}{\|w\|^2}.$$

r is the axis point nearest the origin (rᵀw=0); the full axis is r+αw. Pitch λ is m/rad; translation per turn is 2πλ. At w=0, motion is pure translation: do not divide by \(\|w\|^2\).

Similarly, for a wrench with F≠0:

$$\tau=r\times F+\lambda_FF,\qquad r=\frac{F\times\tau}{\|F\|^2},\quad\lambda_F=\frac{F^T\tau}{\|F\|^2}.$$

This gives the resultant force axis plus a free moment along it. At F=0 only a pure couple remains. Screw geometry describes instantaneous motion; finite displacement uses its exponential.

## 10. The SE(3) exponential

For constant spatial twist:

$$\dot H=\widetilde TH,\qquad H(t)=\exp(\widetilde Tt)H(0).$$

For normalized screw axis \(\hat T=(\hat w;v)\), \(\|\hat w\|=1\) and angle q:

$$\exp(\widetilde{\hat T}q)=\begin{bmatrix}R(q)&V(q)v\\0&1\end{bmatrix},$$

$$R(q)=I+\sin q\,\widetilde{\hat w}+(1-\cos q)\widetilde{\hat w}^{\,2},$$

$$V(q)=qI+(1-\cos q)\widetilde{\hat w}+(q-\sin q)\widetilde{\hat w}^{\,2}.$$

If w=0 and v is a unit prismatic direction, q is linear displacement: R=I, o=vq. A normalized screw axis differs from a physical twist: \(T=\hat T\dot q\).

Code `exp_se3(X)` accepts an **integrated** 6-vector X=(φ;ρ), such as TΔt, and uses

$$o=\left(I+\frac{1-\cos\theta}{\theta^2}\widetilde\phi+\frac{\theta-\sin\theta}{\theta^3}\widetilde\phi^2\right)\rho,\quad\theta=\|\phi\|.$$

Do not substitute velocity without multiplying by time. At φ=0, coefficients have finite limits and o=ρ.

## 11. Fixing a consistent spatial 3R geometry

Use the vertical post and two links in source Figure 5.1. Specify the model axis directions and dimensions explicitly:

- home joint axes are z₀,x₀,x₀;
- axis points: r₁=(0,0,0), r₂=(0,0,ℓ₁), r₃=(0,ℓ₂,ℓ₁);
- links 2 and 3 point along +y₀ at home;
- Ψ₄ origin is o₄⁰(0)=(0,ℓ₂+ℓ₃,ℓ₁), with R₄⁰(0)=I;
- lengths are ℓ₁=0.4 m, ℓ₂=0.35 m, ℓ₃=0.25 m.

After joint 1 rotates, axes 2 and 3 rotate with it; they do not remain fixed world x₀ axes. Listed directions apply to the home pose.

A revolute home axis has linear component r×ŵ:

$$\hat T_1^{0,0}\big|_0=\begin{bmatrix}0\\0\\1\\0\\0\\0\end{bmatrix},\quad\hat T_2^{0,1}\big|_0=\begin{bmatrix}1\\0\\0\\0\\\ell_1\\0\end{bmatrix},\quad\hat T_3^{0,2}\big|_0=\begin{bmatrix}1\\0\\0\\0\\\ell_1\\-\ell_2\end{bmatrix}.$$

For compactness call these **constant home axes** A₁,A₂,A₃, abbreviating \(\hat T_i^{0,i-1}|_{q=0}\) without changing the main notation.

## 12. Product of Exponentials: forward kinematics

$$\boxed{H_4^0(q)=\exp(\widetilde A_1q_1)\exp(\widetilde A_2q_2)\exp(\widetilde A_3q_3)H_4^0(0).}$$

The rightmost matrix is home configuration, not automatically identity. A product of exponentials cannot generally be replaced by an exponential of their sum unless the matrices commute.

An independent local-transform construction of the same geometry:

$$H_4^0=R_z(q_1)\,\mathrm{Trans}_z(\ell_1)\,R_x(q_2)\,\mathrm{Trans}_y(\ell_2)\,R_x(q_3)\,\mathrm{Trans}_y(\ell_3).$$

Here Rₓ,R_z are embedded in 4×4 matrices; successive transforms refer to local frames. Code `direct_chain` is separate from PoE so the check does not repeat its algorithm.

## 13. Space and body Jacobians

Differentiating PoE gives

$$T_4^{0,0}=J_s(q)\dot q,\qquad J_s=\begin{bmatrix}A_1&\operatorname{Ad}_{E_1}A_2&\operatorname{Ad}_{E_1E_2}A_3\end{bmatrix},$$

where \(E_i=\exp(\widetilde A_iq_i)\). Transform each joint axis using preceding joints only.

$$T_4^{4,0}=J_b(q)\dot q,\qquad\boxed{J_b=\operatorname{Ad}_{H_0^4}J_s.}$$

Both Jacobians are 6×3 and represent the same relative twist in different frames. Three drives cannot independently prescribe arbitrary six-component pose velocity.

### Tip-origin velocity Jacobian

The lower three Js rows give v, not ȯ₄⁰. Recover the point-velocity field:

$$\boxed{\dot o_4^0=J_p\dot q,\qquad J_p=\begin{bmatrix}-\widetilde{o_4^0}&I\end{bmatrix}J_s.}$$

At this 3R's home pose, `rank(Js)=3` but `rank(Jp)=2`: two pitch joints produce collinear tip-velocity directions. With nonzero q₃, Jp usually has rank 3. “Singular Jacobian” must specify the task and map.

An analytical Jacobian depends on orientation coordinates, such as Euler angles. Differentiating position gives Jp, not automatically a six-dimensional Euler-angle Jacobian.

### Statics through power

For external wrench row W₄⁰ acting **on the robot**,

$$\mathcal P=W_4^0J_s\dot q=\tau_{\rm ext}^T\dot q,\qquad\tau_{\rm ext}=J_s^T(W_4^0)^T.$$

For W in Ψ₄, use Jb or transform W first. Without other loads, holding actuator torque opposes external generalized torque. Sign depends on the force definition.

## 14. Examples and verification

1. **Frames and power:** `examples.py` computes Pʲ, twist, and wrench in two frames, checking invariance of \(\mathcal P\).
2. **Screw path:** rotation about z through (0.25;0;0) m with pitch 0.08 m/rad. Initial point (0.55;0;0) follows a radius-0.30 m helix.
3. **SO(3) structure:** additive Euler versus exponential steps at h=0.05 s, checking ‖RᵀR−I‖ and det R.
4. **PoE and Jacobians:** compute H₄⁰,Js,Jb,Jp analytically and compare finite differences.
5. **MuJoCo:** set q(t),q̇(t), call `mj_forward`, and compare pose/Jacobians of site `tip`. This is **kinematic playback**, not dynamics or control. XML gravity cannot change prescribed q because `mj_step` is not called.

Direct absolute-path scripts work from any directory; modules run from the repository root. [Instructions](spatial_kinematics/README.en.md).

Slides include draggable 3D scenes, noncommuting rotations, screw parameters, SO(3) drift, 3R joint tuning, v versus ȯ, and power invariance. Browser scenes use analytical kinematics, not MuJoCo.

## 15. Exercises

1. Ψ₂ is rotated +90° around z₁. Express physical vector x₁ in Ψ₂. **Answer:** (0,−1,0).
2. Why Hᵀ≠H⁻¹ although Rᵀ=R⁻¹? **Answer:** inverse translation is −Rᵀo in the upper-right block.
3. Find v for w=(0,0,2) rotating around the line through (0.25;0;0). **Answer:** (0;−0.5;0) m/s.
4. Why is a spatial twist's lower part not tip velocity? Recover \(\dot o=\widetilde wo+v\).
5. Derive the row-W rule from power when Tᵏ=Ad_H Tʲ. **Answer:** Wᵏ=WʲAd_H⁻¹.
6. Find home Jp columns and explain rank 2. **Answer:** (−(ℓ₂+ℓ₃),0,0), (0,0,ℓ₂+ℓ₃), (0,0,ℓ₃).
7. Vary the SO(3) integration step. Why can exponential stepping preserve orthogonality at large steps without guaranteeing accuracy for varying velocity?
8. Compare PoE and local composition at q=(0.4;0.5;−0.7). Compare Js q̇ with vee(ḢH⁻¹), not just one point's path.

## 16. Checking transformations and common mistakes

When working with formulas, check index directions, matrix inverses and vector conventions:

| Source location | Convention here |
|---|---|
| Chapter 4, (4.5): Hᵀ used for inversion | Use H⁻¹; transpose inverts R, not H |
| Chapter 4, Proposition 4: ḢHᵀ and HᵀḢ | ḢH⁻¹ and H⁻¹Ḣ belong to se(3) |
| Chapter 4, wrench transformations | Retain the original **row** convention and derive it from power |
| Chapter 5, (5.5): w̃ and unit axis mixed | Rodrigues uses unit ŵ and a separate angle, or φ with /θ and /θ² coefficients |
| Chapter 5, differing axes/home coordinates | Fix z,x,x; r₂=(0,0,ℓ₁), r₃=(0,ℓ₂,ℓ₁), o₄⁰(0)=(0,ℓ₂+ℓ₃,ℓ₁), consistent with the screw axes and Figure 5.1 |
| Chapter 5, spatial linear velocity | Distinguish lower Js q̇ from tip-origin velocity |

## 17. References and Modern Robotics notation

| These lectures | Modern Robotics |
|---|---|
| Hᵢʲ | Tⱼᵢ, configuration rather than twist |
| Rᵢʲ | Rⱼᵢ |
| Tᵢʲ˒ʲ | V_s, spatial twist |
| Tᵢⁱ˒ʲ | V_b, body twist |
| Wᵢᵏ row | Fₖ column, namely (Wᵢᵏ)ᵀ |
| Aᵢ=\(\hat T_i^{0,i-1}|_0\) | Sᵢ, spatial home axis |

- Kevin M. Lynch, Frank C. Park, **Modern Robotics: Mechanics, Planning, and Control**, Cambridge University Press, 2017. [Official site](https://modernrobotics.northwestern.edu/).
- [Chapter 3: Rigid-Body Motions](https://modernrobotics.northwestern.edu/chapters/chapter3/): §3.2 SO(3), angular velocity, exponential coordinates; §3.3 SE(3), twists/exponentials; §3.4 wrenches.
- [§3.3.3: Exponential Coordinates of Rigid-Body Motion](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-3-exponential-coordinates-of-rigid-body-motion/).
- [§3.4: Wrenches](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-4-wrenches/).
- [Chapter 4: Forward Kinematics](https://modernrobotics.northwestern.edu/chapters/chapter4/): §4.1 Product of Exponentials.
- [Chapter 5: Velocity Kinematics and Statics](https://modernrobotics.northwestern.edu/chapters/chapter5/): §5.1 space/body Jacobians, §5.2 statics, §5.3 singularities.
- MuJoCo: [API mj_forward](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-forward), [mj_jacSite](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite), [passive viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer).

Before using the book, compare indices and wrench row/column conventions in the table: the geometry is the same, the notation differs.

