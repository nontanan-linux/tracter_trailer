# Tractor-Trailer Dynamic Model (5-DOF Lagrangian Formulation)

This repository contains a full dynamic model of a tractor-trailer system derived using the analytical **Lagrangian Mechanics** approach. The system consists of three rigid bodies: the Tractor, the Drawbar (Dolly), and the Trailer. 

This model reduces the system from a constrained 9-DOF Newton-Euler system down to a minimal, unconstrained **5-DOF system**, significantly improving mathematical stability and computational efficiency.

---

## 1. System Kinematics and Coordinates

We define the system using 5 independent generalized coordinates:
$$ q = \begin{bmatrix} x_v & y_v & \theta_v & \theta_d & \theta_t \end{bmatrix}^T $$

Where:
- $(x_v, y_v)$ is the global position of the tractor's Center of Gravity (CG).
- $\theta_v, \theta_d, \theta_t$ are the heading angles of the Tractor, Drawbar, and Trailer, respectively.

### Position Equations
Using the tractor's CG as the base reference, the positions of the Drawbar CG ($x_d, y_d$) and the Trailer CG ($x_t, y_t$) are determined kinematically.
Let $L_{hitch}$ be the distance from Tractor CG to Hitch 1, $L_{bar}$ be the distance from Hitch 1 to Drawbar CG, and $L_{trail}$ be the distance from Hitch 2 to Trailer CG.

**Drawbar CG Position:**
$$ x_d = x_v - L_{hitch} \cos\theta_v - L_{bar} \cos\theta_d $$
$$ y_d = y_v - L_{hitch} \sin\theta_v - L_{bar} \sin\theta_d $$

**Trailer CG Position:**
$$ x_t = x_d - L_{trail} \cos\theta_t = x_v - L_{hitch} \cos\theta_v - L_{bar} \cos\theta_d - L_{trail} \cos\theta_t $$
$$ y_t = y_d - L_{trail} \sin\theta_t = y_v - L_{hitch} \sin\theta_v - L_{bar} \sin\theta_d - L_{trail} \sin\theta_t $$

### Velocity Equations (Time Derivatives)
Taking the time derivative of the positions yields the velocities of each body's CG in the global frame:

**Drawbar Velocities:**
$$ \dot{x}_d = \dot{x}_v + L_{hitch} \dot{\theta}_v \sin\theta_v + L_{bar} \dot{\theta}_d \sin\theta_d $$
$$ \dot{y}_d = \dot{y}_v - L_{hitch} \dot{\theta}_v \cos\theta_v - L_{bar} \dot{\theta}_d \cos\theta_d $$

**Trailer Velocities:**
$$ \dot{x}_t = \dot{x}_v + L_{hitch} \dot{\theta}_v \sin\theta_v + L_{bar} \dot{\theta}_d \sin\theta_d + L_{trail} \dot{\theta}_t \sin\theta_t $$
$$ \dot{y}_t = \dot{y}_v - L_{hitch} \dot{\theta}_v \cos\theta_v - L_{bar} \dot{\theta}_d \cos\theta_d - L_{trail} \dot{\theta}_t \cos\theta_t $$

---

## 2. Kinetic Energy ($T$)

The total kinetic energy of the system is the sum of the kinetic energies of the three rigid bodies:
$$ T = T_v + T_d + T_t $$

Where the kinetic energy of each body includes both translational and rotational components:
$$ T_v = \frac{1}{2}m_v (\dot{x}_v^2 + \dot{y}_v^2) + \frac{1}{2}I_v \dot{\theta}_v^2 $$
$$ T_d = \frac{1}{2}m_d (\dot{x}_d^2 + \dot{y}_d^2) + \frac{1}{2}I_d \dot{\theta}_d^2 $$
$$ T_t = \frac{1}{2}m_t (\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_t \dot{\theta}_t^2 $$

Substituting the velocity equations into $T_d$ and $T_t$ and expanding the squares yields the full kinetic energy equation in terms of the generalized coordinates $q$ and velocities $\dot{q}$. The expanded squared velocity terms reveal the coupling between the bodies:

For example, $\dot{x}_d^2 + \dot{y}_d^2$:
$$ \dot{x}_d^2 + \dot{y}_d^2 = \dot{x}_v^2 + \dot{y}_v^2 + L_{hitch}^2 \dot{\theta}_v^2 + L_{bar}^2 \dot{\theta}_d^2 + 2L_{hitch}(\dot{x}_v\sin\theta_v - \dot{y}_v\cos\theta_v)\dot{\theta}_v + 2L_{bar}(\dot{x}_v\sin\theta_d - \dot{y}_v\cos\theta_d)\dot{\theta}_d + 2L_{hitch} L_{bar}\dot{\theta}_v\dot{\theta}_d\cos(\theta_v - \theta_d) $$

---

## 3. Euler-Lagrange Equations

Since we assume the vehicle operates on a flat plane (or handle slopes as external generalized forces), the potential energy $V = 0$. The Lagrangian is simply $L = T$.

The equations of motion are derived using the Euler-Lagrange formula for each generalized coordinate $q_i$:
$$ \frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial q_i} = Q_i $$

By performing these partial derivatives and time derivatives across all 5 coordinates, we can group the acceleration terms ($\ddot{q}$) into a Mass Matrix $M(q)$, and group the velocity-squared terms ($\dot{q}^2$) into a Coriolis vector $n(q, \dot{q})$:
$$ M(q)\ddot{q} + n(q, \dot{q}) = Q $$

---

## 4. Final Matrix Formulation

### 4.1 Mass Matrix $M(q)$

$$
M = \begin{bmatrix}
(m_v+m_d+m_t) & 0 & (m_d+m_t)L_{hitch}\sin\theta_v & (m_d+m_t)L_{bar}\sin\theta_d & m_t L_{trail}\sin\theta_t \\
0 & (m_v+m_d+m_t) & -(m_d+m_t)L_{hitch}\cos\theta_v & -(m_d+m_t)L_{bar}\cos\theta_d & -m_t L_{trail}\cos\theta_t \\
(m_d+m_t)L_{hitch}\sin\theta_v & -(m_d+m_t)L_{hitch}\cos\theta_v & I_v + (m_d+m_t)L_{hitch}^2 & (m_d+m_t)L_{hitch} L_{bar}\cos(\theta_v-\theta_d) & m_t L_{hitch} L_{trail}\cos(\theta_v-\theta_t) \\
(m_d+m_t)L_{bar}\sin\theta_d & -(m_d+m_t)L_{bar}\cos\theta_d & (m_d+m_t)L_{hitch} L_{bar}\cos(\theta_v-\theta_d) & I_d + (m_d+m_t)L_{bar}^2 & m_t L_{bar} L_{trail}\cos(\theta_d-\theta_t) \\
m_t L_{trail}\sin\theta_t & -m_t L_{trail}\cos\theta_t & m_t L_{hitch} L_{trail}\cos(\theta_v-\theta_t) & m_t L_{bar} L_{trail}\cos(\theta_d-\theta_t) & I_t + m_t L_{trail}^2
\end{bmatrix}
$$

### 4.2 Coriolis and Centrifugal Vector $n(q, \dot{q})$

$$
n = \begin{bmatrix}
(m_d+m_t) L_{hitch} \dot{\theta}_v^2 \cos\theta_v + (m_d+m_t) L_{bar} \dot{\theta}_d^2 \cos\theta_d + m_t L_{trail} \dot{\theta}_t^2 \cos\theta_t \\
(m_d+m_t) L_{hitch} \dot{\theta}_v^2 \sin\theta_v + (m_d+m_t) L_{bar} \dot{\theta}_d^2 \sin\theta_d + m_t L_{trail} \dot{\theta}_t^2 \sin\theta_t \\
-(m_d+m_t) L_{hitch} L_{bar} \dot{\theta}_d^2 \sin(\theta_v-\theta_d) - m_t L_{hitch} L_{trail} \dot{\theta}_t^2 \sin(\theta_v-\theta_t) \\
-(m_d+m_t) L_{hitch} L_{bar} \dot{\theta}_v^2 \sin(\theta_v-\theta_d) + m_t L_{bar} L_{trail} \dot{\theta}_t^2 \sin(\theta_d-\theta_t) \\
-m_t L_{hitch} L_{trail} \dot{\theta}_v^2 \sin(\theta_v-\theta_t) - m_t L_{bar} L_{trail} \dot{\theta}_d^2 \sin(\theta_d-\theta_t)
\end{bmatrix}
$$

### 4.3 Generalized Forces $Q$

The right-hand side vector $Q$ represents the external forces (primarily tire friction) projected onto the generalized coordinates using the **Principle of Virtual Work** ($\delta W = \sum F_i \cdot \delta r_i$).

Let $F_{yf}, F_{yr}, F_{yd}, F_{ytr}$ be the lateral tire friction forces in their respective local body frames.
We rotate these local forces into the global $X-Y$ frame:
- $Q_{xv} = \sum F_{X,global}$
- $Q_{yv} = \sum F_{Y,global}$

For the rotational coordinates, we project the global forces onto the moment arms relative to the CGs and hitch points:
- $Q_{\theta_v} = l_f F_{yf\_in\_v} - l_r F_{yr\_in\_v} - L_{hitch} F_{yd\_in\_v} - L_{hitch} F_{yt\_in\_v}$
- $Q_{\theta_d} = - L_{bar} F_{yd\_in\_d} - L_{bar} F_{yt\_in\_d}$
- $Q_{\theta_t} = - (l_{ft} + l_{rt}) F_{yt\_in\_t}$

*(Note: Subscripts like `_in_v` denote the global force vector projected into the local Y-axis of the respective body. For example, the trailer tire forces $F_{yt}$ apply torque not just on the trailer, but they are also felt by the drawbar and the tractor through the rigid hitches.)*

---

## 5. Implementation Details

- `dynamic_model.py`: Implements the calculations of $M$, $n$, and $Q$, and solves for the accelerations using $ \ddot{q} = M^{-1}(Q - n) $.
- `simulate_dynamic.py`: Runs a time-stepping simulation using a semi-implicit Euler integration method. To ensure numerical stability given the extreme stiffness of the tire slip models, the physics loop runs at a sub-stepped rate of 1ms.