# Dynamic Model of a Towing Vehicle

This document is prepared to present the mathematical derivation of a planar dynamic model for a towing vehicle and full trailer system (Tractor + Drawbar + Trailer Body), focusing on the application of **Lagrangian Dynamics** to formulate the equations of motion.

---

## 0. Schematic and Coordinate Systems

### 0.1 Tracter with Drawbar Trailler Diagram
This model consists of 3 main rigid bodies connected by revolute joints (Hitch Joints):
1. **Tractor**: Has mass $m$, Center of Gravity (CG) at coordinates $(x_0, y_0)$ in the global frame, and a heading (Yaw Angle) of $\theta_0$.
2. **Drawbar / Dolly**: Has mass $m_d$, acting as the front wheels of the trailer that can steer. It is connected to the rear of the tractor at hitch $H_1$ with a drawbar length $L_{bar}$, and has a heading angle of $\theta_1$.
3. **Trailer Body**: Has mass $m_t$, connected exactly at the axle of the drawbar (no overhang, $l_{rd} = 0$), making the structure act like a single truck with the drawbar as its front wheels, and has a heading angle of $\theta_2$.

#### Parameters Explanation
| Symbol | Category | Description |
| :---: | :--- | :--- |
| **$(x_0, y_0)$** | Coordinate / Position | Tractor Rear Axle Center (Main reference point) |
| **$(x_d, y_d)$** | Coordinate / Position | Drawbar Front Axle Center |
| **$(x_t, y_t)$** | Coordinate / Position | Trailer Center of Gravity |
| **$H_1$** | Joint / Connection | Hitch 1 (Tractor to Drawbar) |
| **$H_2$** | Joint / Connection | Hitch 2 (Drawbar to Trailer Body, located exactly at the drawbar front axle) |
| **$L_{bar}$** | Geometry / Dimension | Drawbar Arm Length (From $H_1$ to front axle) |
| **$d_h$** | Geometry / Dimension | Tractor Rear Overhang (From rear axle to $H_1$) |
| **$l_{ft}, l_{rt}$** | Geometry / Dimension | Trailer Front/Rear Lengths (From CG to front and rear axles) |
| **$\theta_0$** | Angle / Orientation | Tractor Yaw Angle (Relative to global X axis) |
| **$\theta_1$** | Angle / Orientation | Drawbar Yaw Angle (Relative to global X axis) |
| **$\theta_2$** | Angle / Orientation | Trailer Yaw Angle (Relative to global X axis) |
| **$\delta$** | Angle / Orientation | Front Wheel Steer Angle (Relative to longitudinal vehicle axis) |

---

## 1. Origin and Principles of the Method

The dynamic analysis of multi-body systems connected by joints can be performed using energy methods to reduce the complexity of calculating internal reaction forces.

### 1.2 Lagrangian Dynamics for Multi-Body Systems
This system consists of 3 rigid bodies (Tractor, Drawbar, Trailer Body) with a total of 9 generalized coordinates:

$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T \in \mathbb{R}^9$$

#### Euler-Lagrange Equation
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_j}\right) - \frac{\partial L}{\partial q_j} = Q_j + F_{h,j}$$

### 1.3 Derivation of Linear Velocities in Global Coordinates
The velocity of the Drawbar only depends on the distance $L_{bar}$, because the Trailer is considered as a single vehicle with hitch $H_2$ attached exactly at the Drawbar axle position ($l_{rd} = 0$).

**1. Tractor:**

$$\dot{x}_0 = v_0 \cos\theta_0$$

$$\dot{y}_0 = v_0 \sin\theta_0$$

**2. Drawbar / Dolly:**
The coordinate $(x_d, y_d)$ is connected to the rear of the tractor at a distance $L_{bar}$:

$$x_d = x_0 - d_h \cos\theta_0 - L_{bar} \cos\theta_1$$

$$y_d = y_0 - d_h \sin\theta_0 - L_{bar} \sin\theta_1$$

Taking the derivative with respect to time:

$$\dot{x}_d = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + L_{bar} \dot{\theta}_1 \sin\theta_1$$

$$\dot{y}_d = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - L_{bar} \dot{\theta}_1 \cos\theta_1$$

**3. Trailer Body:**
The coordinate $(x_t, y_t)$ connects exactly at the Drawbar axle:

$$x_t = x_d - l_{ft} \cos\theta_2 = x_0 - d_h \cos\theta_0 - L_{bar} \cos\theta_1 - l_{ft} \cos\theta_2$$

$$y_t = y_d - l_{ft} \sin\theta_2 = y_0 - d_h \sin\theta_0 - L_{bar} \sin\theta_1 - l_{ft} \sin\theta_2$$

Taking the derivative with respect to time:

$$\dot{x}_t = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + L_{bar} \dot{\theta}_1 \sin\theta_1 + l_{ft} \dot{\theta}_2 \sin\theta_2$$

$$\dot{y}_t = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - L_{bar} \dot{\theta}_1 \cos\theta_1 - l_{ft} \dot{\theta}_2 \cos\theta_2$$

### 1.5 Kinetic and Potential Energy

$$
T = \left[ \frac{1}{2}m(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_z\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_d(\dot{x}_d^2 + \dot{y}_d^2) + \frac{1}{2}I_{zd}\dot{\theta}_1^2 \right] + \left[ \frac{1}{2}m_t(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_{zt}\dot{\theta}_2^2 \right]
$$

$$V = 0 \implies L = T$$

---

## 2. Derivation of the Equations of Motion using Generalized Coordinates

### 2.1 Deriving the Lagrangian Equation $L$
Let the Generalized Coordinates vector of the system be:

$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T$$

From the Lagrangian equation $L = T - V$ (where $V=0$, thus $L=T$), we differentiate with respect to each generalized coordinate and its velocity to form the left-hand side of the Euler-Lagrange equation $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i}$:

**1. Tractor Coordinates: $q_1 \dots q_3$**

$$
q_1 = x_0: \quad \frac{\partial L}{\partial \dot{x}_0} = m \dot{x}_0, \quad \frac{\partial L}{\partial x_0} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_0}\right) - \frac{\partial L}{\partial x_0} = m \ddot{x}_0
$$

$$
q_2 = y_0: \quad \frac{\partial L}{\partial \dot{y}_0} = m \dot{y}_0, \quad \frac{\partial L}{\partial y_0} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_0}\right) - \frac{\partial L}{\partial y_0} = m \ddot{y}_0
$$

$$
q_3 = \theta_0: \quad \frac{\partial L}{\partial \dot{\theta}_0} = I_z \dot{\theta}_0, \quad \frac{\partial L}{\partial \theta_0} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_0}\right) - \frac{\partial L}{\partial \theta_0} = I_z \ddot{\theta}_0
$$

**2. Drawbar / Dolly Coordinates: $q_4 \dots q_6$**

$$
q_4 = x_d: \quad \frac{\partial L}{\partial \dot{x}_d} = m_d \dot{x}_d, \quad \frac{\partial L}{\partial x_d} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_d}\right) - \frac{\partial L}{\partial x_d} = m_d \ddot{x}_d
$$

$$
q_5 = y_d: \quad \frac{\partial L}{\partial \dot{y}_d} = m_d \dot{y}_d, \quad \frac{\partial L}{\partial y_d} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_d}\right) - \frac{\partial L}{\partial y_d} = m_d \ddot{y}_d
$$

$$
q_6 = \theta_1: \quad \frac{\partial L}{\partial \dot{\theta}_1} = I_{zd} \dot{\theta}_1, \quad \frac{\partial L}{\partial \theta_1} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_1}\right) - \frac{\partial L}{\partial \theta_1} = I_{zd} \ddot{\theta}_1
$$

**3. Trailer Body Coordinates: $q_7 \dots q_9$**

$$
q_7 = x_t: \quad \frac{\partial L}{\partial \dot{x}_t} = m_t \dot{x}_t, \quad \frac{\partial L}{\partial x_t} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_t}\right) - \frac{\partial L}{\partial x_t} = m_t \ddot{x}_t
$$

$$
q_8 = y_t: \quad \frac{\partial L}{\partial \dot{y}_t} = m_t \dot{y}_t, \quad \frac{\partial L}{\partial y_t} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_t}\right) - \frac{\partial L}{\partial y_t} = m_t \ddot{y}_t
$$

$$
q_9 = \theta_2: \quad \frac{\partial L}{\partial \dot{\theta}_2} = I_{zt} \dot{\theta}_2, \quad \frac{\partial L}{\partial \theta_2} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_2}\right) - \frac{\partial L}{\partial \theta_2} = I_{zt} \ddot{\theta}_2
$$

### 2.2 Generalized Forces ($Q_i$)
The generalized force $Q_i$ is the sum of non-conservative external forces (excluding hitch reaction forces $\lambda$) acting on the generalized coordinate $q_i$. For this vehicle, this includes Tire Forces and Traction Forces acting on each body, defined as follows:

**1. Tractor: $Q_1 \dots Q_3$**
*   $Q_1 = Q_{x0} = \sum F_{X0}$ (Sum of external forces in global X axis)
*   $Q_2 = Q_{y0} = \sum F_{Y0}$ (Sum of external forces in global Y axis)
*   $Q_3 = Q_{\theta0} = \sum M_{z0}$ (Sum of external moments around Tractor CG)

**2. Drawbar / Dolly: $Q_4 \dots Q_6$**
*   $Q_4 = Q_{xd} = \sum F_{Xd}$ (Sum of external forces in global X axis)
*   $Q_5 = Q_{yd} = \sum F_{Yd}$ (Sum of external forces in global Y axis)
*   $Q_6 = Q_{\theta1} = \sum M_{zd}$ (Sum of external moments around Drawbar front axle center)

**3. Trailer Body: $Q_7 \dots Q_9$**
*   $Q_7 = Q_{xt} = \sum F_{Xt}$ (Sum of external forces in global X axis)
*   $Q_8 = Q_{yt} = \sum F_{Yt}$ (Sum of external forces in global Y axis)
*   $Q_9 = Q_{\theta2} = \sum M_{zt}$ (Sum of external moments around Trailer Body CG)

### 2.3 Inertial Equations of Motion
Formulating the acceleration equations incorporating the hitch reaction forces $\lambda_1, \lambda_2$ (at $H_1$) and $\lambda_3, \lambda_4$ (at $H_2$, located exactly at the Drawbar front axle):

1.  **Tractor ($x_0, y_0, \theta_0$):**

    $$
    m\ddot{x_0} = Q_{x0} - \lambda_1
    $$

    $$
    m\ddot{y_0} = Q_{y0} - \lambda_2
    $$

    $$
    I_z\ddot{\theta_0} = Q_{\theta0} - d_h \sin\theta_0 \lambda_1 + d_h \cos\theta_0 \lambda_2
    $$

2.  **Drawbar ($x_d, y_d, \theta_1$):**

    $$m_d \ddot{x_d} = Q_{xd} + \lambda_1 - \lambda_3$$

    $$m_d \ddot{y_d} = Q_{yd} + \lambda_2 - \lambda_4$$

    $$I_{zd} \ddot{\theta_1} = Q_{\theta1} - L_{bar} \sin\theta_1 \lambda_1 + L_{bar} \cos\theta_1 \lambda_2$$

    *(Note: Forces $\lambda_3, \lambda_4$ from the Trailer act exactly at the Drawbar CG, thus creating no moment around $\theta_1$)*
3.  **Trailer Body ($x_t, y_t, \theta_2$):**

    $$m_t \ddot{x_t} = Q_{xt} + \lambda_3$$

    $$m_t \ddot{y}_t = Q_{yt} + \lambda_4$$

    $$I_{zt} \ddot{\theta}_2 = Q_{\theta2} - l_{ft} \sin\theta_2 \lambda_3 + l_{ft} \cos\theta_2 \lambda_4$$

### 2.4 Body-Fixed Equations of Motion
Transforming accelerations $\ddot{x}, \ddot{y}$ into the body-fixed frame $(\dot{v}_x - v_y r)$ and decomposing tire forces:

#### 1. Tractor

$$\text{Longitudinal:} \quad m(\dot{v}_x - v_y r) = F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1}$$

$$\text{Lateral:} \quad m(\dot{v}_y + v_x r) = F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1}$$

$$\text{Yaw:} \quad I_z \dot{r} = l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} + d_h F_{hy1}$$

#### 2. Drawbar
Defining relative angles $\Delta\theta_1 = \theta_0 - \theta_1$ and $\Delta\theta_2 = \theta_1 - \theta_2$:

$$\text{Longitudinal:} \quad m_d(\dot{v}_{xd} - v_{yd} r_d) = F_{xd} + F_{hx1}\cos\Delta\theta_1 - F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2$$

$$\text{Lateral:} \quad m_d(\dot{v}_{yd} + v_{xd} r_d) = F_{yd} + F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2$$

$$\text{Yaw:} \quad I_{zd} \dot{r}_d = L_{bar} (F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1)$$

#### 3. Trailer Body

$$\text{Longitudinal:} \quad m_t(\dot{v}_{xt} - v_{yt} r_t) = F_{xt} + F_{hx2}$$

$$\text{Lateral:} \quad m_t(\dot{v}_{yt} + v_{xt} r_t) = F_{ytr} + F_{hy2}$$

$$\text{Yaw:} \quad I_{zt} \dot{r}_t = l_{ft} F_{hy2} - l_{rt} F_{ytr}$$

### 2.5 Hitch Acceleration Constraints
Differentiating the joint velocity constraints to obtain the acceleration constraints ($\ddot{g} = 0$) at points $H_1$ and $H_2$, resulting in 4 equations:

**Constraint at Hitch 1 ($H_1$):**

$$\dot{v}_{xd} - \dot{v}_x \cos\Delta\theta_1 + \dot{v}_y \sin\Delta\theta_1 - d_h \dot{r} \sin\Delta\theta_1 = (r - r_d) \left[-v_x \sin\Delta\theta_1 - (v_y - d_h r) \cos\Delta\theta_1\right]$$

$$\dot{v}_{yd} + L_{bar} \dot{r}_d - \dot{v}_x \sin\Delta\theta_1 - \dot{v}_y \cos\Delta\theta_1 + d_h \dot{r} \cos\Delta\theta_1 = (r - r_d) \left[v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1\right]$$

**Constraint at Hitch 2 ($H_2$ connects exactly at the Drawbar axle, meaning $l_{rd}=0$):**

$$\dot{v}_{xt} - \dot{v}_{xd} \cos\Delta\theta_2 + \dot{v}_{yd} \sin\Delta\theta_2 = (r_d - r_t) \left[-v_{xd} \sin\Delta\theta_2 - v_{yd} \cos\Delta\theta_2\right]$$

$$\dot{v}_{yt} + l_{ft} \dot{r}_t - \dot{v}_{xd} \sin\Delta\theta_2 - \dot{v}_{yd} \cos\Delta\theta_2 = (r_d - r_t) \left[v_{xd} \cos\Delta\theta_2 - v_{yd} \sin\Delta\theta_2\right]$$

### 2.6 Matrix Formulation

$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} = Q $$

#### 1. Acceleration and Velocity Vectors

*   $\ddot{q} = [\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xd}, \dot{v}_{yd}, \dot{r}_d, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t]^T$

*   $\dot{q} = [v_x, v_y, r, v_{xd}, v_{yd}, r_d, v_{xt}, v_{yt}, r_t]^T$

#### 2. Mass and Inertia Matrix $M(q)$

$$ M(q) = \text{diag}(m, m, I_z, m_d, m_d, I_{zd}, m_t, m_t, I_{zt}) $$

#### 3. Coriolis and Centrifugal Force Matrix $C(q, \dot{q})$

$$
C(q, \dot{q}) = \begin{bmatrix}
0 & -m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -m_d r_d & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_d r_d & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & -m_t r_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & m_t r_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

#### 4. Generalized Force Vector $Q$

$$
Q = \begin{bmatrix}
F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1} \\
F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1} \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} + d_h F_{hy1} \\
F_{xd} + F_{hx1}\cos\Delta\theta_1 - F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2 \\
F_{yd} + F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2 \\
L_{bar} (F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1) \\
F_{xt} + F_{hx2} \\
F_{ytr} + F_{hy2} \\
l_{ft} F_{hy2} - l_{rt} F_{ytr}
\end{bmatrix}
$$

#### 5. Full Expanded Equation

$$
\begin{bmatrix}
m & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & m & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & I_z & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_d & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & m_d & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & I_{zd} & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & m_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & m_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & I_{zt}
\end{bmatrix}
\begin{bmatrix}
\dot{v}_x \\ \dot{v}_y \\ \dot{r} \\ \dot{v}_{xd} \\ \dot{v}_{yd} \\ \dot{r}_d \\ \dot{v}_{xt} \\ \dot{v}_{yt} \\ \dot{r}_t
\end{bmatrix}
+
\begin{bmatrix}
0 & -m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -m_d r_d & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_d r_d & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & -m_t r_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & m_t r_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
v_x \\ v_y \\ r \\ v_{xd} \\ v_{yd} \\ r_d \\ v_{xt} \\ v_{yt} \\ r_t
\end{bmatrix}
=
\begin{bmatrix}
F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1} \\
F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1} \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} + d_h F_{hy1} \\
F_{xd} + F_{hx1}\cos\Delta\theta_1 - F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2 \\
F_{yd} + F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2 \\
L_{bar} (F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1) \\
F_{xt} + F_{hx2} \\
F_{ytr} + F_{hy2} \\
l_{ft} F_{hy2} - l_{rt} F_{ytr}
\end{bmatrix}
$$
