import numpy as np

class TractorTrailerModel:
    def __init__(self, L0, dh, L1, L2, dh2, L3, L4, dt=0.1, max_steering_angle=np.radians(30), max_drawbar_angle=np.radians(30), max_velocity=5.0):
        """
        Initialize the kinematic model parameters for Tractor + Trailer 1 + Trailer 2.
        
        Args:
            L0 (float): Wheelbase of the tractor.
            dh (float): Distance from tractor rear axle to hitch 1.
            L1 (float): Drawbar 1 length (Hitch 1 to Dolly 1).
            L2 (float): Trailer 1 length (Dolly 1 to Axle 1).
            dh2 (float): Distance from Trailer 1 rear axle to hitch 2.
            L3 (float): Drawbar 2 length (Hitch 2 to Dolly 2).
            L4 (float): Trailer 2 length (Dolly 2 to Axle 2).
            dt (float): Time step.
            max_steering_angle (float): Max steering angle.
            max_drawbar_angle (float): Max relative angle for drawbars.
            max_velocity (float): Max velocity.
        """
        self.L0 = L0
        self.dh = dh
        self.L1 = L1
        self.L2 = L2
        self.dh2 = dh2
        self.L3 = L3
        self.L4 = L4
        self.dt = dt
        self.max_steering_angle = max_steering_angle
        self.max_drawbar_angle = max_drawbar_angle
        self.max_velocity = max_velocity

    def get_state_derivative(self, state, v0, delta):
        """
        Calculate the time derivative of the state.
        
        Args:
            state (list or np.array): [x0, y0, theta0, theta1, theta2, theta3, theta4]
            v0 (float): Longitudinal velocity of the tractor.
            delta (float): Steering angle of the tractor.
            
        Returns:
            np.array: Time derivative [dx0, dy0, dtheta0, dtheta1, dtheta2, dtheta3, dtheta4]
        """
        # Compute full kinematic vector
        q_kin = self.get_kinematic_vector(state, v0, delta)
        
        # Extract state derivatives: [dx0, dy0, dtheta0, dtheta1, dtheta2, dtheta3, dtheta4]
        # Indices in q_kin: 0, 1, 2, 4, 6, 8, 10
        dx = q_kin[[0, 1, 2, 4, 6, 8, 10]]
        
        return dx

    def get_kinematic_vector(self, state, v0, delta):
        """
        Calculate the full 11-element kinematic vector including linear velocities.
        
        Args:
            state (list or np.array): [x0, y0, theta0, theta1, theta2, theta3, theta4]
            v0 (float): Longitudinal velocity of the tractor.
            delta (float): Steering angle of the tractor.
            
        Returns:
            np.array: [dx0, dy0, dtheta0, v1, dtheta1, v2, dtheta2, v3, dtheta3, v4, dtheta4]
        """
        x0, y0, theta0, theta1, theta2, theta3, theta4 = state
        
        # Pre-compute angle differences
        t01 = theta0 - theta1
        t12 = theta1 - theta2
        t23 = theta2 - theta3
        t34 = theta3 - theta4
        
        # Input vector u' = [v0, dtheta0] (Tractor Velocity Vector)
        dtheta0 = (v0 / self.L0) * np.tan(delta)
        u_prime = np.array([v0, dtheta0])
        
        # Initialize Matrix S (11x2)
        S = np.zeros((11, 2))
        
        # Row 0 (dx0): [cos(theta0), 0]
        S[0, 0] = np.cos(theta0)
        
        # Row 1 (dy0): [sin(theta0), 0]
        S[1, 0] = np.sin(theta0)
        
        # Row 2 (dtheta0): [0, 1]
        S[2, 1] = 1.0
        
        # Row 3 (v1): [cos(t01), dh*sin(t01)]
        S[3, 0] = np.cos(t01)
        S[3, 1] = self.dh * np.sin(t01)
        
        # Row 4 (dtheta1): [1/L1 * sin(t01), -dh/L1 * cos(t01)]
        S[4, 0] = (1 / self.L1) * np.sin(t01)
        S[4, 1] = -(self.dh / self.L1) * np.cos(t01)
        
        # Row 5 (v2): [cos(t01)cos(t12), dh*sin(t01)cos(t12)]
        S[5, 0] = np.cos(t01) * np.cos(t12)
        S[5, 1] = self.dh * np.sin(t01) * np.cos(t12)
        
        # Row 6 (dtheta2): [1/L2 * cos(t01)sin(t12), dh/L2 * sin(t01)sin(t12)]
        S[6, 0] = (1 / self.L2) * np.cos(t01) * np.sin(t12)
        S[6, 1] = (self.dh / self.L2) * np.sin(t01) * np.sin(t12)
        
        # Row 7 (v3)
        # Term M from previous derivation logic, now expanded
        # M = cos(t12)cos(t23) + (dh2/L2)sin(t12)sin(t23)
        M = np.cos(t12) * np.cos(t23) + (self.dh2 / self.L2) * np.sin(t12) * np.sin(t23)
        S[7, 0] = np.cos(t01) * M
        S[7, 1] = self.dh * np.sin(t01) * M
        
        # Row 8 (dtheta3)
        # Term N from previous derivation logic, now expanded
        # N = cos(t12)sin(t23) - (dh2/L2)sin(t12)cos(t23)
        N = np.cos(t12) * np.sin(t23) - (self.dh2 / self.L2) * np.sin(t12) * np.cos(t23)
        S[8, 0] = (1 / self.L3) * np.cos(t01) * N
        S[8, 1] = (self.dh / self.L3) * np.sin(t01) * N
        
        # Row 9 (v4)
        # v4 = v3 * cos(t34) -> Row 9 = Row 7 * cos(t34)
        S[9, 0] = S[7, 0] * np.cos(t34)
        S[9, 1] = S[7, 1] * np.cos(t34)
        
        # Row 10 (dtheta4)
        # dtheta4 = v3/L4 * sin(t34) -> Row 10 = Row 7 * sin(t34)/L4
        S[10, 0] = S[7, 0] * (np.sin(t34) / self.L4)
        S[10, 1] = S[7, 1] * (np.sin(t34) / self.L4)
        
        # Compute kinematic vector
        q_kin = S @ u_prime
        
        return q_kin

    def update(self, state, v0, delta):
        """
        Update the state using Euler integration.
        """
        # Clamp Steering Angle
        delta = np.clip(delta, -self.max_steering_angle, self.max_steering_angle)
        
        # Clamp Velocity
        v0 = np.clip(v0, -self.max_velocity, self.max_velocity)
        
        state = np.array(state)
        derivative = self.get_state_derivative(state, v0, delta)
        new_state = state + derivative * self.dt
        
        # Helper to normalize and clamp relative angle
        def clamp_angle(theta_curr, theta_prev, max_angle):
            rel = theta_curr - theta_prev
            while rel > np.pi: rel -= 2*np.pi
            while rel < -np.pi: rel += 2*np.pi
            
            if rel > max_angle:
                return theta_prev + max_angle
            elif rel < -max_angle:
                return theta_prev - max_angle
            return theta_curr

        # Clamp Drawbar 1 (theta1 vs theta0)
        new_state[3] = clamp_angle(new_state[3], new_state[2], self.max_drawbar_angle)
        
        # Clamp Drawbar 2 (theta3 vs theta2)
        new_state[5] = clamp_angle(new_state[5], new_state[4], self.max_drawbar_angle)
        
        return new_state

    def get_coordinates(self, state):
        """
        Calculate coordinates of all key points for visualization.
        """
        x0, y0, theta0, theta1, theta2, theta3, theta4 = state
        
        # Tractor Rear Axle
        p0 = np.array([x0, y0])
        # Tractor Front Axle
        p0_f = p0 + self.L0 * np.array([np.cos(theta0), np.sin(theta0)])
        
        # Hitch 1
        h1 = p0 - self.dh * np.array([np.cos(theta0), np.sin(theta0)])
        
        # Dolly 1 Axle
        p1 = h1 - self.L1 * np.array([np.cos(theta1), np.sin(theta1)])
        
        # Trailer 1 Axle
        p2 = p1 - self.L2 * np.array([np.cos(theta2), np.sin(theta2)])
        
        # Hitch 2
        h2 = p2 - self.dh2 * np.array([np.cos(theta2), np.sin(theta2)])
        
        # Dolly 2 Axle
        p3 = h2 - self.L3 * np.array([np.cos(theta3), np.sin(theta3)])
        
        # Trailer 2 Axle
        p4 = p3 - self.L4 * np.array([np.cos(theta4), np.sin(theta4)])
        
        return p0, p0_f, h1, p1, p2, h2, p3, p4
