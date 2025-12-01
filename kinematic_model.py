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
        
        # --- Tractor ---
        dx0 = v0 * np.cos(theta0)
        dy0 = v0 * np.sin(theta0)
        dtheta0 = (v0 / self.L0) * np.tan(delta)
        
        # --- Trailer 1 ---
        # Hitch 1 Velocity
        # v_h1_perp = v0 * sin(theta0 - theta1) - dh * dtheta0 * cos(theta0 - theta1)
        term1 = v0 * np.sin(theta0 - theta1)
        term2 = self.dh * dtheta0 * np.cos(theta0 - theta1)
        dtheta1 = (1 / self.L1) * (term1 - term2)
        
        # Dolly 1 Velocity (v1)
        v1 = v0 * np.cos(theta0 - theta1) + self.dh * dtheta0 * np.sin(theta0 - theta1)
        
        # Trailer 1 Rotation
        dtheta2 = (v1 / self.L2) * np.sin(theta1 - theta2)
        
        # --- Trailer 2 ---
        # Hitch 2 is at distance dh2 behind Trailer 1 axle (P2)
        # Similar to Tractor->Hitch1, but source is Trailer 1 (v2, theta2)
        # Velocity of Trailer 1 Axle is v2 (along theta2).
        # Wait, v2 is velocity of P2.
        # v2 = v1 * cos(theta1 - theta2) (Projected along trailer)
        # Actually, let's re-derive v2 properly.
        # Velocity of P1 is v1 along theta1.
        # Velocity of P2 (Trailer 1 Axle) along theta2 is v2.
        # v2 = v1 * np.cos(theta1 - theta2)
        
        v2 = v1 * np.cos(theta1 - theta2)
        
        # Hitch 2 Velocity components relative to Drawbar 2 (theta3)
        # Source is P2 with velocity v2 along theta2, and rotation dtheta2.
        # Hitch 2 is dh2 behind P2.
        # This is exactly analogous to Tractor (v0, theta0, dtheta0, dh) -> Drawbar 1 (theta1).
        # Replace: v0->v2, theta0->theta2, dtheta0->dtheta2, dh->dh2, theta1->theta3, L1->L3.
        
        term1_2 = v2 * np.sin(theta2 - theta3)
        term2_2 = self.dh2 * dtheta2 * np.cos(theta2 - theta3)
        dtheta3 = (1 / self.L3) * (term1_2 - term2_2)
        
        # Dolly 2 Velocity (v3)
        v3 = v2 * np.cos(theta2 - theta3) + self.dh2 * dtheta2 * np.sin(theta2 - theta3)
        
        # Trailer 2 Rotation
        # Replace: v1->v3, L2->L4, theta1->theta3, theta2->theta4
        dtheta4 = (v3 / self.L4) * np.sin(theta3 - theta4)
        
        return np.array([dx0, dy0, dtheta0, dtheta1, dtheta2, dtheta3, dtheta4])

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
