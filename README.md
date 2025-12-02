# Kinematic Model of Tractor with Two Drawbar Trailers

This project implements a kinematic model and simulation for a **Multi-Trailer System** consisting of a tractor and two drawbar trailers. The model uses standard bicycle kinematics extended to a chain of rigid bodies.

## 1. System Description

The system consists of five main components connected in a chain:
1.  **Tractor**: The towing vehicle with front-wheel steering.
2.  **Drawbar 1**: Connects the tractor to the first trailer.
3.  **Trailer 1**: The first cargo unit.
4.  **Drawbar 2**: Connects the first trailer to the second trailer.
5.  **Trailer 2**: The second cargo unit.

### Kinematic Diagram
![Kinematic Diagram](kinematic_diagram_full.png)

---

## 2. System Kinematics Vector

The system kinematics is defined by **11 variables**:
$$ \mathbf{q}_{kin} = [\dot{x}_0, \dot{y}_0, \dot{\theta}_0, v_1, \dot{\theta}_1, v_2, \dot{\theta}_2, v_3, \dot{\theta}_3, v_4, \dot{\theta}_4]^T $$

| Variable | Description | Unit |
| :--- | :--- | :--- |
| $\dot{x}_0, \dot{y}_0$ | Velocity of the Tractor's rear axle center (World Frame) | m/s |
| $\dot{\theta}_0$ | Angular Velocity of the Tractor | rad/s |
| $v_1$ | Longitudinal Velocity of Dolly 1 | m/s |
| $\dot{\theta}_1$ | Angular Velocity of Drawbar 1 | rad/s |
| $v_2$ | Longitudinal Velocity of Trailer 1 | m/s |
| $\dot{\theta}_2$ | Angular Velocity of Trailer 1 | rad/s |
| $v_3$ | Longitudinal Velocity of Dolly 2 | m/s |
| $\dot{\theta}_3$ | Angular Velocity of Drawbar 2 | rad/s |
| $v_4$ | Longitudinal Velocity of Trailer 2 | m/s |
| $\dot{\theta}_4$ | Angular Velocity of Trailer 2 | rad/s |

**Inputs**:
*   $v_0$: Longitudinal velocity of the Tractor.
*   $\delta$: Steering angle of the Tractor's front wheels.

---

## 3. System Parameters

| Parameter | Symbol | Value (Default) | Description |
| :--- | :--- | :--- | :--- |
| **Tractor** | $L_0$ | 2.0 m | Wheelbase |
| | $d_h$ | 0.55 m | Hitch 1 offset (behind rear axle) |
| **Trailer 1** | $L_1$ | 1.0 m | Drawbar 1 Length |
| | $L_2$ | 1.2 m | Trailer 1 Length (Dolly to Axle) |
| | $d_{h2}$ | 0.5 m | Hitch 2 offset (behind rear axle) |
| **Trailer 2** | $L_3$ | 1.0 m | Drawbar 2 Length |
| | $L_4$ | 1.2 m | Trailer 2 Length (Dolly to Axle) |
| **General** | $W$ | 1.5 m | Track Width (for visualization) |

---

## 4. Mathematical Model

The equations of motion are derived assuming **no slip** conditions (non-holonomic constraints) for all wheels.

### 4.1 Tractor Kinematics
The tractor follows the standard kinematic bicycle model:
$$ \dot{x}_0 = v_0 \cos\theta_0 $$
$$ \dot{y}_0 = v_0 \sin\theta_0 $$
$$ \dot{\theta}_0 = \frac{v_0}{L_0} \tan\delta $$

### 4.2 Trailer 1 Kinematics
The motion of the first trailer is driven by the velocity of **Hitch 1** ($H_1$).

**Hitch 1 Velocity**:
$$ v_{hx} = v_0 \cos\theta_0 + d_h \dot{\theta}_0 \sin\theta_0 $$
$$ v_{hy} = v_0 \sin\theta_0 - d_h \dot{\theta}_0 \cos\theta_0 $$

> **Derivation Note**:
> The signs differ because of the derivatives of the trigonometric functions.
> *   For $x_h = x_0 - d_h \cos\theta_0$: The derivative of $\cos\theta_0$ is $-\sin\theta_0 \cdot \dot{\theta}_0$. The two negatives cancel out $\rightarrow + d_h \dot{\theta}_0 \sin\theta_0$.
> *   For $y_h = y_0 - d_h \sin\theta_0$: The derivative of $\sin\theta_0$ is $\cos\theta_0 \cdot \dot{\theta}_0$. The negative sign remains $\rightarrow - d_h \dot{\theta}_0 \cos\theta_0$.

**Drawbar 1 Rotation ($\dot{\theta}_1$)**:
Driven by the hitch velocity component perpendicular to the drawbar:
$$ \dot{\theta}_1 = \frac{1}{L_1} \left( v_0 \sin(\theta_0 - \theta_1) - d_h \dot{\theta}_0 \cos(\theta_0 - \theta_1) \right) $$

**Trailer 1 Rotation ($\dot{\theta}_2$)**:
Driven by the velocity of the Dolly 1 axle ($v_1$) pulling the trailer:
$$ v_1 = v_0 \cos(\theta_0 - \theta_1) + d_h \dot{\theta}_0 \sin(\theta_0 - \theta_1) $$
$$ \dot{\theta}_2 = \frac{v_1}{L_2} \sin(\theta_1 - \theta_2) $$

**Linear Velocities**:
*   **Dolly 1 Velocity ($v_1$)**:
    $$ v_1 = v_0 \cos(\theta_0 - \theta_1) + d_h \dot{\theta}_0 \sin(\theta_0 - \theta_1) $$
*   **Trailer 1 Axle Velocity ($v_2$)**:
    $$ v_2 = v_1 \cos(\theta_1 - \theta_2) $$

### 4.3 Trailer 2 Kinematics
The motion of the second trailer is driven by the velocity of **Hitch 2** ($H_2$), located at the rear of Trailer 1.

**Hitch 2 Velocity**:
$$ v_{h2,\perp} = v_2 \sin(\theta_2 - \theta_3) - d_{h2} \dot{\theta}_2 \cos(\theta_2 - \theta_3) $$
where $v_2 = v_1 \cos(\theta_1 - \theta_2)$ is the velocity of Trailer 1's axle.

**Drawbar 2 Rotation ($\dot{\theta}_3$)**:
$$ \dot{\theta}_3 = \frac{1}{L_3} \left( v_2 \sin(\theta_2 - \theta_3) - d_{h2} \dot{\theta}_2 \cos(\theta_2 - \theta_3) \right) $$

**Trailer 2 Rotation ($\dot{\theta}_4$)**:
Driven by the velocity of the Dolly 2 axle ($v_3$):
$$ v_3 = v_2 \cos(\theta_2 - \theta_3) + d_{h2} \dot{\theta}_2 \sin(\theta_2 - \theta_3) $$
$$ \dot{\theta}_4 = \frac{v_3}{L_4} \sin(\theta_3 - \theta_4) $$

**Linear Velocities**:
*   **Dolly 2 Velocity ($v_3$)**:
    $$ v_3 = v_2 \cos(\theta_2 - \theta_3) + d_{h2} \dot{\theta}_2 \sin(\theta_2 - \theta_3) $$
*   **Trailer 2 Axle Velocity ($v_4$)**:
    $$ v_4 = v_3 \cos(\theta_3 - \theta_4) $$

---

## 5. Matrix Form

The system kinematics can be expressed in the matrix form:
where $\mathbf{u}_{tractor} = [v_0, \dot{\theta}_0]^T$ is the tractor's velocity vector.
The angular velocity $\dot{\theta}_0$ is determined by the inputs $v_0$ and steering angle $\delta$:
$$ \dot{\theta}_0 = \frac{v_0}{L_0} \tan\delta $$

$$
\begin{bmatrix}
\dot{x}_0 \\ \dot{y}_0 \\ \dot{\theta}_0 \\ v_1 \\ \dot{\theta}_1 \\ v_2 \\ \dot{\theta}_2 \\ v_3 \\ \dot{\theta}_3 \\ v_4 \\ \dot{\theta}_4
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_0 & 0 \\
\sin\theta_0 & 0 \\
0 & 1 \\
\cos(\theta_0 - \theta_1) & d_h \sin(\theta_0 - \theta_1) \\
\frac{1}{L_1} \sin(\theta_0 - \theta_1) & -\frac{d_h}{L_1} \cos(\theta_0 - \theta_1) \\
\cos(\theta_0 - \theta_1)\cos(\theta_1 - \theta_2) & d_h \sin(\theta_0 - \theta_1)\cos(\theta_1 - \theta_2) \\
\frac{1}{L_2} \cos(\theta_0 - \theta_1) \sin(\theta_1 - \theta_2) & \frac{d_h}{L_2} \sin(\theta_0 - \theta_1) \sin(\theta_1 - \theta_2) \\
\cos(\theta_0 - \theta_1) \left[ \cos(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) + \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) \right] & d_h \sin(\theta_0 - \theta_1) \left[ \cos(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) + \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) \right] \\
\frac{1}{L_3} \cos(\theta_0 - \theta_1) \left[ \cos(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) - \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) \right] & \frac{d_h}{L_3} \sin(\theta_0 - \theta_1) \left[ \cos(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) - \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) \right] \\
\cos(\theta_0 - \theta_1) \cos(\theta_3 - \theta_4) \left[ \cos(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) + \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) \right] & d_h \sin(\theta_0 - \theta_1) \cos(\theta_3 - \theta_4) \left[ \cos(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) + \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) \right] \\
\frac{1}{L_4} \cos(\theta_0 - \theta_1) \sin(\theta_3 - \theta_4) \left[ \cos(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) + \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) \right] & \frac{d_h}{L_4} \sin(\theta_0 - \theta_1) \sin(\theta_3 - \theta_4) \left[ \cos(\theta_1 - \theta_2)\cos(\theta_2 - \theta_3) + \frac{d_{h2}}{L_2}\sin(\theta_1 - \theta_2)\sin(\theta_2 - \theta_3) \right]
\end{bmatrix}
\begin{bmatrix}
v_0 \\ \dot{\theta}_0
\end{bmatrix}
$$

---

## 6. Coordinate Calculation (Forward Kinematics)

The global coordinates $(x, y)$ of key points are calculated from the state vector for visualization:

1.  **Tractor Front Axle ($P_{0,f}$)**:
    $$ P_{0,f} = P_0 + L_0 \begin{bmatrix} \cos\theta_0 \\ \sin\theta_0 \end{bmatrix} $$

2.  **Hitch 1 ($H_1$)**:
    $$ H_1 = P_0 - d_h \begin{bmatrix} \cos\theta_0 \\ \sin\theta_0 \end{bmatrix} $$

3.  **Trailer 1**:
    *   **Dolly 1 ($P_1$)**:
        $$ P_1 = H_1 - L_1 \begin{bmatrix} \cos\theta_1 \\ \sin\theta_1 \end{bmatrix} $$
    *   **Axle 1 ($P_2$)**:
        $$ P_2 = P_1 - L_2 \begin{bmatrix} \cos\theta_2 \\ \sin\theta_2 \end{bmatrix} $$

4.  **Hitch 2 ($H_2$)**:
    $$ H_2 = P_2 - d_{h2} \begin{bmatrix} \cos\theta_2 \\ \sin\theta_2 \end{bmatrix} $$

5.  **Trailer 2**:
    *   **Dolly 2 ($P_3$)**:
        $$ P_3 = H_2 - L_3 \begin{bmatrix} \cos\theta_3 \\ \sin\theta_3 \end{bmatrix} $$
    *   **Axle 2 ($P_4$)**:
        $$ P_4 = P_3 - L_4 \begin{bmatrix} \cos\theta_4 \\ \sin\theta_4 \end{bmatrix} $$

---

## 7. Constraints

*   **Steering Limit**: $\delta \in [-30^\circ, 30^\circ]$
*   **Drawbar Limits**: Relative angle between units $\in [-30^\circ, 30^\circ]$
*   **Velocity Limit**: $v_0 \in [-5.0, 5.0]$ m/s

---

## 8. Usage

### Run Simulation
```bash
python3 simulate.py
```
*   Displays an interactive animation window.
*   Close the window to save the result to `simulation_full.gif`.
*   **Configuration**: Edit `SAVE_ANIMATION` in `simulate.py` to toggle saving.

### Generate Diagram
```bash
python3 create_diagram.py
```
*   Generates `kinematic_diagram_full.png`.
*   Supports command-line arguments for dimensions (e.g., `--L0`, `--L1`, etc.).

---

## 9. Simulation Result

Running `simulate.py` produces an animation of the vehicle trajectory.

![Simulation](simulation_full.gif)
