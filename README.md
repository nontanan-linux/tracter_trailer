# Kinematic Model of Tractor with Drawbar Trailer

This project implements a kinematic model and simulation for a **Tractor-Trailer system** consisting of a tractor, a dolly (drawbar), and a semitrailer. The model is based on standard bicycle model kinematics extended to a multi-body system.

## System Description

The system is modeled as three rigid bodies connected by pivot points (hitches):
1.  **Tractor**: The towing vehicle with front-wheel steering.
2.  **Dolly (Drawbar)**: A tow bar connected to the tractor's hitch, supporting the front of the trailer.
3.  **Trailer**: The main cargo unit connected to the dolly.

### Kinematic Diagram

The diagram below illustrates the geometric parameters and state variables.

![Kinematic Diagram](kinematic_diagram_full.png)

### State Variables
The system state is defined by 5 variables: $q = [x_0, y_0, \theta_0, \theta_1, \theta_2]$
*   $(x_0, y_0)$: Position of the Tractor's rear axle center (World Frame).
*   $\theta_0$: Rotation of the Tow Tractor relative to World Coordinate.
*   $\theta_1$: Angle of the Drawbar (acts as the steering angle for the trailer).
*   $\theta_2$: Rotation of the Trailer relative to World Coordinate.
*   $\delta$: Steering Angle (of the Tractor's front wheels).

### Parameters
*   $L_0 = 2.0$ m: Tractor wheelbase.
*   $d_h$: Distance from Tractor rear axle to Hitch (approx 0.55 m).
*   $L_1 = 1.2$ m: Length of the Drawbar (Hitch to Dolly Axle).
*   $L_2 = 1.2$ m: Length of the Trailer (Dolly Axle to Trailer Axle).
*   $W = 1.5$ m: Track width (distance between wheels).
*   **Vehicle Box Dimensions**:
    *   Tractor: $2.8 \times 1.5$ m (Length x Width).
    *   Trailer: $2.0 \times 1.5$ m (Length x Width).
*   $d_{h2} = 0.15$ m: Trailer tail extension (distance from Trailer rear to rear hitch).

### Inputs
*   $v_0$: Longitudinal velocity of the Tractor.
*   $\delta$: Steering angle of the Tractor's front wheels.

### Constraints
*   **Tractor Steering Limit**: $\delta \in [-30^\circ, 30^\circ]$.
*   **Drawbar Steering Limit**: Relative angle between Tractor and Drawbar $\in [-30^\circ, 30^\circ]$.
*   **Tractor Velocity Limit**: $v_0 \in [-5.0, 5.0]$ m/s (Default).

## Mathematical Model

The equations of motion are derived assuming **no slip** conditions (non-holonomic constraints) for all wheels. The system is viewed as a chain of rigid bodies.

### 1. Tractor Kinematics
The tractor is modeled as a kinematic bicycle model.
*   **Rear Axle Velocity**: The velocity vector at the rear axle $(x_0, y_0)$ is directed along the tractor's heading $\theta_0$.
    $$ \dot{x}_0 = v_0 \cos\theta_0 $$
    $$ \dot{y}_0 = v_0 \sin\theta_0 $$
*   **Yaw Rate**: Determined by the steering angle $\delta$ and wheelbase $L_0$.
    $$ \dot{\theta}_0 = \frac{v_0}{L_0} \tan\delta $$

### 2. Hitch Velocity
The hitch is located at a distance $d_h$ behind the tractor's rear axle. Its position is:
$$ x_h = x_0 - d_h \cos\theta_0 $$
$$ y_h = y_0 - d_h \sin\theta_0 $$

Differentiating with respect to time gives the velocity of the hitch $(v_{hx}, v_{hy})$:
$$ v_{hx} = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 = v_0 \cos\theta_0 + d_h \dot{\theta}_0 \sin\theta_0 $$
$$ v_{hy} = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 = v_0 \sin\theta_0 - d_h \dot{\theta}_0 \cos\theta_0 $$

### 3. Drawbar (Dolly) Kinematics
The drawbar connects the hitch $(x_h, y_h)$ to the dolly axle $(x_1, y_1)$. The length is $L_1$.
The constraint is that the velocity of the hitch *perpendicular* to the drawbar induces rotation $\dot{\theta}_1$.
The velocity of the hitch projected onto the direction perpendicular to the drawbar ($-\sin\theta_1, \cos\theta_1$) is:
$$ v_{h,\perp} = -v_{hx} \sin\theta_1 + v_{hy} \cos\theta_1 $$

Substituting $v_{hx}, v_{hy}$:
$$ v_{h,\perp} = -(v_0 \cos\theta_0 + d_h \dot{\theta}_0 \sin\theta_0)\sin\theta_1 + (v_0 \sin\theta_0 - d_h \dot{\theta}_0 \cos\theta_0)\cos\theta_1 $$
$$ v_{h,\perp} = v_0 (\sin\theta_0 \cos\theta_1 - \cos\theta_0 \sin\theta_1) - d_h \dot{\theta}_0 (\cos\theta_0 \cos\theta_1 + \sin\theta_0 \sin\theta_1) $$
Using trig identities $\sin(A-B)$ and $\cos(A-B)$:
$$ v_{h,\perp} = v_0 \sin(\theta_0 - \theta_1) - d_h \dot{\theta}_0 \cos(\theta_0 - \theta_1) $$

The angular velocity of the drawbar is this perpendicular velocity divided by the length $L_1$:
$$ \dot{\theta}_1 = \frac{1}{L_1} \left( v_0 \sin(\theta_0 - \theta_1) - d_h \dot{\theta}_0 \cos(\theta_0 - \theta_1) \right) $$

### 4. Dolly Axle Velocity ($v_1$)
The velocity of the dolly axle $(x_1, y_1)$ is the component of the hitch velocity *parallel* to the drawbar.
$$ v_1 = v_{hx} \cos\theta_1 + v_{hy} \sin\theta_1 $$
$$ v_1 = (v_0 \cos\theta_0 + d_h \dot{\theta}_0 \sin\theta_0)\cos\theta_1 + (v_0 \sin\theta_0 - d_h \dot{\theta}_0 \cos\theta_0)\sin\theta_1 $$
$$ v_1 = v_0 (\cos\theta_0 \cos\theta_1 + \sin\theta_0 \sin\theta_1) + d_h \dot{\theta}_0 (\sin\theta_0 \cos\theta_1 - \cos\theta_0 \sin\theta_1) $$
$$ v_1 = v_0 \cos(\theta_0 - \theta_1) + d_h \dot{\theta}_0 \sin(\theta_0 - \theta_1) $$

### 5. Trailer Kinematics
The trailer is pulled by the dolly axle. The trailer wheelbase is $L_2$.
Similar to the tractor, the change in trailer angle $\dot{\theta}_2$ depends on the velocity of the pulling point (dolly axle) perpendicular to the trailer orientation.
However, since the dolly axle is a pivot, we can view it as:
$$ \dot{\theta}_2 = \frac{v_1}{L_2} \sin(\theta_1 - \theta_2) $$
*Note: This assumes the dolly wheels steer with $\theta_1$ and the trailer body follows.*

## Code Structure

*   `kinematic_model.py`: Contains the `TractorTrailerModel` class implementing the differential equations.
*   `simulate.py`: Runs a time-stepping simulation (Euler integration) with a sample control input (sine wave steering) and generates an animation.
*   `create_diagram.py`: Generates the schematic diagram of the system.

## Simulation Result

Running `simulate.py` produces an animation of the vehicle trajectory.

![Simulation](simulation_full.gif)

## Usage

1.  **Run Simulation**:
    ```bash
    python3 simulate.py
    ```
    This will generate `simulation_full.gif`.

2.  **Generate Diagram**:
    ```bash
    python3 create_diagram.py
    ```
    This will generate `kinematic_diagram_full.png`.
    
    You can also specify parameters via command line arguments:
    ```bash
    python3 create_diagram.py --L0 2.5 --L1 1.5 --L2 2.0 --W 1.5
    ```
    Available arguments:
    *   `--L0`: Tractor Wheelbase
    *   `--L1`: Drawbar Length
    *   `--L2`: Trailer Wheelbase
    *   `--W`: Track Width
    *   `--trailer_len`: Trailer Body Length
    *   `--tail_ext`: Tail Extension Length
