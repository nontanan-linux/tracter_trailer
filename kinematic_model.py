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
        x0, y0, theta0, theta1, theta2, theta3, theta4 = state
        
        # Pre-compute angle differences
        theta01 = theta0 - theta1
        theta12 = theta1 - theta2
        theta23 = theta2 - theta3
        theta34 = theta3 - theta4
        
        # Input vector u' = [v0, dtheta0]
        dtheta0 = (v0 / self.L0) * np.tan(delta)
        u_prime = np.array([v0, dtheta0])
        
        # Initialize Matrix S (7x2)
        S = np.zeros((7, 2))
        
        # Row 0 (x0): [cos(theta0), 0]
        S[0, 0] = np.cos(theta0)
        
        # Row 1 (y0): [sin(theta0), 0]
        S[1, 0] = np.sin(theta0)
        
        # Row 2 (theta0): [0, 1]
        S[2, 1] = 1.0
        
        # Row 3 (theta1)
        # dtheta1 = (1/L1) * (v0*sin(theta01) - dh*dtheta0*cos(theta01))
        S[3, 0] = (1 / self.L1) * np.sin(theta01)
        S[3, 1] = -(self.dh / self.L1) * np.cos(theta01)
        
        # Helper for v1 coefficients (v1 = v0*c01 + dtheta0*dh*s01)
        # v1 = C_v1_v0 * v0 + C_v1_w0 * dtheta0
        C_v1_v0 = np.cos(theta01)
        C_v1_w0 = self.dh * np.sin(theta01)
        
        # Row 4 (theta2)
        # dtheta2 = (v1/L2) * sin(theta12)
        S[4, 0] = (C_v1_v0 / self.L2) * np.sin(theta12)
        S[4, 1] = (C_v1_w0 / self.L2) * np.sin(theta12)
        
        # Helper for v2 coefficients (v2 = v1 * cos(theta12))
        C_v2_v0 = C_v1_v0 * np.cos(theta12)
        C_v2_w0 = C_v1_w0 * np.cos(theta12)
        
        # Row 5 (theta3)
        # dtheta3 = (1/L3) * (v2*sin(theta23) - dh2*dtheta2*cos(theta23))
        # Substitute v2 and dtheta2 (from S[4])
        S[5, 0] = (1 / self.L3) * (C_v2_v0 * np.sin(theta23) - self.dh2 * S[4, 0] * np.cos(theta23))
        S[5, 1] = (1 / self.L3) * (C_v2_w0 * np.sin(theta23) - self.dh2 * S[4, 1] * np.cos(theta23))
        
        # Helper for v3 coefficients
        # v3 = v2*cos(theta23) + dh2*dtheta2*sin(theta23)
        C_v3_v0 = C_v2_v0 * np.cos(theta23) + self.dh2 * S[4, 0] * np.sin(theta23)
        C_v3_w0 = C_v2_w0 * np.cos(theta23) + self.dh2 * S[4, 1] * np.sin(theta23)
        
        # Row 6 (theta4)
        # dtheta4 = (v3/L4) * sin(theta34)
        S[6, 0] = (C_v3_v0 / self.L4) * np.sin(theta34)
        S[6, 1] = (C_v3_w0 / self.L4) * np.sin(theta34)
        
        # Compute state derivative
        dx = S @ u_prime
        
        return dx

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
