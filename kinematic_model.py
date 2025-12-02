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
        Calculate the full 11-element kinematic vector using recursive matrix formulation.
        
        Args:
            state (list or np.array): [x0, y0, theta0, theta1, theta2, theta3, theta4]
            v0 (float): Longitudinal velocity of the tractor.
            delta (float): Steering angle of the tractor.
            
        Returns:
            np.array: [dx0, dy0, dtheta0, v1, dtheta1, v2, dtheta2, v3, dtheta3, v4, dtheta4]
        """
        x0, y0, theta0, theta1, theta2, theta3, theta4 = state
        
        # 1. Tractor Input
        dtheta0 = (v0 / self.L0) * np.tan(delta)
        v0_vec = np.array([v0, dtheta0]) # [v0, dtheta0]
        
        # Helper: Transform A (Tractor/Trailer -> Drawbar)
        def transform_A(v_prev, theta_prev, theta_curr, L_bar, d_h_prev):
            delta_theta = theta_prev - theta_curr
            c, s = np.cos(delta_theta), np.sin(delta_theta)
            # M_A matrix
            M_A = np.array([
                [c, d_h_prev * s],
                [(1/L_bar) * s, -(d_h_prev/L_bar) * c]
            ])
            return M_A @ v_prev

        # Helper: Transform B (Drawbar -> Trailer)
        def transform_B(v_prev, theta_prev, theta_curr, L_trl):
            delta_theta = theta_prev - theta_curr
            c, s = np.cos(delta_theta), np.sin(delta_theta)
            # M_B matrix
            M_B = np.array([
                [c, 0],
                [(1/L_trl) * s, 0]
            ])
            return M_B @ v_prev

        # 2. Recursive Calculation
        # Unit 1: Drawbar 1 (from Tractor)
        v1_vec = transform_A(v0_vec, theta0, theta1, self.L1, self.dh) # [v1, dtheta1]
        
        # Unit 2: Trailer 1 (from Drawbar 1)
        v2_vec = transform_B(v1_vec, theta1, theta2, self.L2) # [v2, dtheta2]
        
        # Unit 3: Drawbar 2 (from Trailer 1)
        v3_vec = transform_A(v2_vec, theta2, theta3, self.L3, self.dh2) # [v3, dtheta3]
        
        # Unit 4: Trailer 2 (from Drawbar 2)
        v4_vec = transform_B(v3_vec, theta3, theta4, self.L4) # [v4, dtheta4]
        
        # 3. Assemble Full Vector
        # [dx0, dy0]
        dx0 = v0 * np.cos(theta0)
        dy0 = v0 * np.sin(theta0)
        
        q_kin = np.array([
            dx0, dy0, 
            v0_vec[1],      # dtheta0
            v1_vec[0], v1_vec[1], # v1, dtheta1
            v2_vec[0], v2_vec[1], # v2, dtheta2
            v3_vec[0], v3_vec[1], # v3, dtheta3
            v4_vec[0], v4_vec[1]  # v4, dtheta4
        ])
        
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
