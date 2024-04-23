# Mathematical Foundations and Control Theory

Kestrel implements an aerospace-grade mathematical stack for non-linear flight dynamics, SO(3) Lie group attitude estimation, Blade Element Momentum Theory (BEMT), and cascaded feedback control.

---

## 1. 6-DOF Non-Linear Rigid Body Dynamics

The full motion of the aerial vehicle is governed by the Newton-Euler equations expressed in the body-fixed Forward-Right-Down (FRD) frame:

### Translational Dynamics
The rate of change of linear momentum in a rotating body frame satisfies:

$$m \left( \dot{\mathbf{v}}_b + \boldsymbol{\omega}_b \times \mathbf{v}_b \right) = \mathbf{F}_{\text{total},b}$$

$$\dot{\mathbf{v}}_b = \frac{1}{m} \mathbf{F}_{\text{total},b} - \boldsymbol{\omega}_b \times \mathbf{v}_b$$

where:
- $m$ is the total vehicle mass (1.45 kg).
- $\mathbf{v}_b = [u, v, w]^T$ is the linear velocity in the body frame.
- $\boldsymbol{\omega}_b = [p, q, r]^T$ is the angular rate vector (roll, pitch, yaw rates).
- $\mathbf{F}_{\text{total},b} = \mathbf{F}_{\text{thrust},b} + \mathbf{F}_{\text{drag},b} + \mathbf{R}^T \mathbf{F}_{\text{gravity},n}$.
- $\mathbf{R}$ is the rotation matrix from body frame to inertial NED frame.

### Rotational Dynamics
The rate of change of angular momentum satisfies Euler's rotational equation:

$$\mathbf{I} \dot{\boldsymbol{\omega}}_b + \boldsymbol{\omega}_b \times (\mathbf{I} \boldsymbol{\omega}_b) = \boldsymbol{\tau}_{\text{total},b} + \boldsymbol{\tau}_{\text{gyro}}$$

$$\dot{\boldsymbol{\omega}}_b = \mathbf{I}^{-1} \left( \boldsymbol{\tau}_{\text{total},b} - \boldsymbol{\omega}_b \times (\mathbf{I} \boldsymbol{\omega}_b) - \sum_{i=1}^{4} \mathbf{I}_{r} (\boldsymbol{\omega}_b \times \mathbf{z}_b) \Omega_i (-1)^i \right)$$

where:
- $\mathbf{I} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the principal diagonal inertia tensor:
  $$I_{xx} = 0.0142 \text{ kg}\cdot\text{m}^2, \quad I_{yy} = 0.0154 \text{ kg}\cdot\text{m}^2, \quad I_{zz} = 0.0268 \text{ kg}\cdot\text{m}^2$$
- $\boldsymbol{\tau}_{\text{total},b}$ is the net torque produced by rotor thrust differentials and blade aerodynamic drag.
- $\boldsymbol{\tau}_{\text{gyro}}$ represents the gyroscopic precession moment exerted by spinning rotor disks.

---

## 2. SO(3) Lie Group Kinematics and Numerical Integration

### Quaternion Kinematics
Vehicle attitude is represented by a unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^T \in S^3$. The kinematic differential equation is:

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\Omega}_b$$

where $\boldsymbol{\Omega}_b = [0, p, q, r]^T$ is the pure quaternion of body angular rates, and $\otimes$ denotes Hamilton quaternion multiplication.

### Lie Group Exponential Map Integration
Naive Euler integration of quaternions causes norm degradation $(\|\mathbf{q}\| \neq 1.0)$ under high angular acceleration, leading to scale divergence. Kestrel employs the exact closed-form exponential map over $SO(3)$:

Given angular velocity $\boldsymbol{\omega}$ over timestep $\Delta t$, let $\Delta \boldsymbol{\theta} = \boldsymbol{\omega} \Delta t$ and $\theta = \|\Delta \boldsymbol{\theta}\|$.

For $\theta > 10^{-6}$:

$$\exp\left(\frac{1}{2} \Delta \boldsymbol{\theta}\right) = \left[ \cos\left(\frac{\theta}{2}\right), \frac{\Delta \boldsymbol{\theta}}{\theta} \sin\left(\frac{\theta}{2}\right) \right]^T$$

For $\theta \le 10^{-6}$, Taylor series expansion prevents division-by-zero singularities:

$$\exp\left(\frac{1}{2} \Delta \boldsymbol{\theta}\right) \approx \left[ 1 - \frac{\theta^2}{8}, \frac{\Delta \boldsymbol{\theta}}{2} \left(1 - \frac{\theta^2}{24}\right) \right]^T$$

The updated attitude is strictly norm-preserving:

$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \exp\left(\frac{1}{2} \boldsymbol{\omega} \Delta t\right)$$

### Geodesic Attitude Error on SO(3)
To eliminate unwinding and gimbal lock issues associated with Euler angles, attitude tracking errors are computed directly on the Lie algebra $\mathfrak{so}(3)$:

$$\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{current}}^{-1} \otimes \mathbf{q}_{\text{target}}$$

Enforcing the shortest-path geodesic condition (antipodal disambiguation):

$$\text{if } q_{\text{err}, w} < 0 \implies \mathbf{q}_{\text{err}} \leftarrow -\mathbf{q}_{\text{err}}$$

The 3-axis rotation vector error is extracted via:

$$\mathbf{e}_R = 2 \cdot \text{sgn}(q_w) \cdot [q_x, q_y, q_z]^T$$

---

## 3. Cascaded Flight Control Architecture

Kestrel employs a four-tier cascaded architecture operating across multiple update frequencies:

```
+-----------------------------------------------------------------------------------+
|                        CASCADED FLIGHT CONTROLLER LOOPS                           |
+-----------------------------------------------------------------------------------+
|                                                                                   |
|  [ Waypoint Error ]                                                               |
|          |                                                                        |
|          v                                                                        |
|  +--------------------+                                                           |
|  | Position Loop      |  10 Hz Outer Loop                                         |
|  | P + Velocity Feedf |  Output: Desired Velocity Vector v_des                    |
|  +-------+------------+                                                           |
|          |                                                                        |
|          v                                                                        |
|  +--------------------+                                                           |
|  | Velocity Loop      |  50 Hz Translational Loop                                 |
|  | PID + Accel Clamp  |  Output: Desired Inertial Acceleration a_des              |
|  +-------+------------+                                                           |
|          |                                                                        |
|          v                                                                        |
|  +--------------------+                                                           |
|  | Attitude Mapper    |  Geometric SO(3) Projection                               |
|  | Accel -> Tilt (R/P)|  Output: Target Quaternion q_des & Collective Thrust T    |
|  +-------+------------+                                                           |
|          |                                                                        |
|          v                                                                        |
|  +--------------------+                                                           |
|  | Attitude Loop      |  250 Hz Rotational Loop                                   |
|  | Geodesic P-Control |  Output: Desired Angular Rates (P_des, Q_des, R_des)       |
|  +-------+------------+                                                           |
|          |                                                                        |
|          v                                                                        |
|  +--------------------+                                                           |
|  | Angular Rate Loop  |  500 Hz Inner Torque Loop                                 |
|  | High-Rate PID      |  Output: Normalized Body Moments (tau_x, tau_y, tau_z)    |
|  +-------+------------+                                                           |
|          |                                                                        |
|          v                                                                        |
|  +--------------------+                                                           |
|  | Quad-X Motor Mixer |  Attitude Priority Desaturation Matrix                    |
|  | Geometric Alloc    |  Output: Motor Demands [M1, M2, M3, M4] in [0.0, 1.0]     |
|  +--------------------+                                                           |
+-----------------------------------------------------------------------------------+
```

### Quad-X Geometric Motor Allocation
For a Quad-X airframe with motor arms configured at $\pm 45^\circ$, the allocation matrix mapping total thrust $T$ and control moments $[\tau_\phi, \tau_\theta, \tau_\psi]^T$ into individual motor commands $[m_1, m_2, m_3, m_4]^T$ is:

$$\begin{bmatrix} m_1 \\ m_2 \\ m_3 \\ m_4 \end{bmatrix} = \begin{bmatrix} 1 & -1 &  1 & -1 \\ 1 &  1 & -1 & -1 \\ 1 &  1 &  1 &  1 \\ 1 & -1 & -1 &  1 \end{bmatrix} \begin{bmatrix} T \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix}$$

- $m_1$: Front-Right (Counter-Clockwise)
- $m_2$: Rear-Left (Counter-Clockwise)
- $m_3$: Front-Left (Clockwise)
- $m_4$: Rear-Right (Clockwise)

### Attitude Priority Anti-Windup Desaturation
When large control inputs cause any motor demand to exceed physical ceiling ($m_i > 1.0$) or floor ($m_i < 0.0$), naive clipping distorts torque balance, causing catastrophic loss of attitude control. Kestrel applies an attitude-priority desaturation:

1. Identify maximum requested motor demand:
   $$m_{\max} = \max(m_1, m_2, m_3, m_4)$$
2. If $m_{\max} > 1.0$, reduce the collective throttle component $T$ by $(m_{\max} - 1.0)$:
   $$m_i \leftarrow m_i - (m_{\max} - 1.0) \quad \forall i \in \{1, 2, 3, 4\}$$
3. Clamp lower boundary to idle floor:
   $$m_i \leftarrow \max(0.05, m_i)$$

This guarantees that differential torques required for roll and pitch stability are strictly preserved, sacrificing climb rate momentarily to prevent uncontrolled tumbling.

---

## 4. Aerodynamic Propulsion and Blade Element Momentum Theory (BEMT)

Each rotor is modeled using Blade Element Momentum Theory, combining momentum conservation across the rotor disk with 2D airfoil lift and drag characteristics:

### Blade Element Inflow and Thrust
For a rotor disk of radius $R$, blade chord $c$, and blade count $N_b$, the blade solidity is:

$$\sigma = \frac{N_b c}{\pi R}$$

The dynamic pressure at 75% blade radius is $q_{0.75} = \frac{1}{2} \rho (\Omega \cdot 0.75 R)^2$.

The effective angle of attack accounting for induced downwash velocity $v_i$ is:

$$\alpha_{\text{eff}} = \theta_{\text{pitch}} - \arctan\left(\frac{V_{\infty,z} + v_i}{0.75 \Omega R}\right)$$

Thrust coefficient $C_T$ integrated along the blade radius yields:

$$C_T = \frac{1}{2} \sigma C_{L,\alpha} \alpha_{\text{eff}} \left( \frac{1}{3} \right)$$

$$T = C_T \rho A (\Omega R)^2$$

### Induced Downwash Inflow Solver
The induced velocity $v_i$ is solved iteratively via Newton-Raphson on the momentum conservation residual:

$$f(v_i) = v_i - \frac{T}{2 \rho A \sqrt{V_{\infty,z}^2 + v_i^2}} = 0$$

### Cheeseman-Bennett Ground Effect (IGE)
When hovering near ground $(z \le 2R)$, the physical surface constrains downwash expansion, reducing induced velocity and boosting thrust:

$$\frac{T_{\text{IGE}}}{T_{\text{OGE}}} = \frac{1}{1 - \left(\frac{R}{4z}\right)^2}$$

Kestrel smoothly clamps the ground effect multiplier to a physical ceiling of 1.45 at touchdown clearance.

---

## 5. Atmospheric and Turbulence Modeling

### Dryden Continuous Turbulence Model
Atmospheric gust disturbances are generated using Dryden spatial spectra conforming to MIL-F-8785C:

$$\Phi_u(\Omega) = \sigma_u^2 \frac{2 L_u}{\pi} \frac{1}{1 + (L_u \Omega)^2}$$

$$\Phi_w(\Omega) = \sigma_w^2 \frac{L_w}{\pi} \frac{1 + 3 (L_w \Omega)^2}{(1 + (L_w \Omega)^2)^2}$$

Spatial turbulence velocity components $[u_g, v_g, w_g]$ are filtered and injected directly into the aerodynamic relative wind equations on every physics substep.
