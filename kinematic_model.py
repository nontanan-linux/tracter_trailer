import numpy as np

class TractorTrailerModel:
    def __init__(self, L0, dh, L1, L2, dh2, L3, L4, dh3, L5, L6, dh4, L7, L8, dt=0.1, max_steering_angle=np.radians(30), max_drawbar_angle=np.radians(30), max_velocity=5.0):
        """
        Initialize the kinematic model parameters for Tractor + 4 Trailers.
        """
        self.L0 = L0
        self.dh = dh
        self.L1 = L1
        self.L2 = L2
        self.dh2 = dh2
        self.L3 = L3
        self.L4 = L4
        self.dh3 = dh3
        self.L5 = L5
        self.L6 = L6
        self.dh4 = dh4
        self.L7 = L7
        self.L8 = L8
        self.dt = dt
        self.max_steering_angle = max_steering_angle
        self.max_drawbar_angle = max_drawbar_angle
        self.max_velocity = max_velocity

    def get_state_derivative(self, state, v0, delta):
        """
        Calculate the time derivative of the state.
        
        Args:
            state (list or np.array): [x0, y0, theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8]
            
        Returns:
            np.array: Time derivative [dx0, dy0, dtheta0, dtheta1, dtheta2, dtheta3, dtheta4, dtheta5, dtheta6, dtheta7, dtheta8]
        """
        # Compute full kinematic vector
        q_kin = self.get_kinematic_vector(state, v0, delta)
        
        # Extract state derivatives
        # q_kin indices:
        # 0,1,2: dx0, dy0, dtheta0
        # 4: dtheta1
        # 6: dtheta2
        # 8: dtheta3
        # 10: dtheta4
        # 12: dtheta5
        # 14: dtheta6
        # 16: dtheta7
        # 18: dtheta8
        dx = q_kin[[0, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18]]
        
        return dx

    def get_kinematic_vector(self, state, v0, delta):
        """
        Calculate the full 19-element kinematic vector using recursive matrix formulation.
        """
        x0, y0, theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8 = state
        
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
        v1_vec = transform_A(v0_vec, theta0, theta1, self.L1, self.dh)
        # Unit 2: Trailer 1 (from Drawbar 1)
        v2_vec = transform_B(v1_vec, theta1, theta2, self.L2)
        
        # Unit 3: Drawbar 2 (from Trailer 1)
        v3_vec = transform_A(v2_vec, theta2, theta3, self.L3, self.dh2)
        # Unit 4: Trailer 2 (from Drawbar 2)
        v4_vec = transform_B(v3_vec, theta3, theta4, self.L4)
        
        # Unit 5: Drawbar 3 (from Trailer 2)
        v5_vec = transform_A(v4_vec, theta4, theta5, self.L5, self.dh3)
        # Unit 6: Trailer 3 (from Drawbar 3)
        v6_vec = transform_B(v5_vec, theta5, theta6, self.L6)
        
        # Unit 7: Drawbar 4 (from Trailer 3)
        v7_vec = transform_A(v6_vec, theta6, theta7, self.L7, self.dh4)
        # Unit 8: Trailer 4 (from Drawbar 4)
        v8_vec = transform_B(v7_vec, theta7, theta8, self.L8)
        
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
            v4_vec[0], v4_vec[1], # v4, dtheta4
            v5_vec[0], v5_vec[1], # v5, dtheta5
            v6_vec[0], v6_vec[1], # v6, dtheta6
            v7_vec[0], v7_vec[1], # v7, dtheta7
            v8_vec[0], v8_vec[1]  # v8, dtheta8
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
        
        # Clamp Drawbar 3 (theta5 vs theta4)
        new_state[7] = clamp_angle(new_state[7], new_state[6], self.max_drawbar_angle)
        
        # Clamp Drawbar 4 (theta7 vs theta6)
        new_state[9] = clamp_angle(new_state[9], new_state[8], self.max_drawbar_angle)
        
        return new_state

    def get_coordinates(self, state):
        """
        Calculate coordinates of all key points for visualization.
        """
        x0, y0, theta0, theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8 = state
        
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
        
        # Hitch 3
        h3 = p4 - self.dh3 * np.array([np.cos(theta4), np.sin(theta4)])
        
        # Dolly 3 Axle
        p5 = h3 - self.L5 * np.array([np.cos(theta5), np.sin(theta5)])
        # Trailer 3 Axle
        p6 = p5 - self.L6 * np.array([np.cos(theta6), np.sin(theta6)])
        
        # Hitch 4
        h4 = p6 - self.dh4 * np.array([np.cos(theta6), np.sin(theta6)])
        
        # Dolly 4 Axle
        p7 = h4 - self.L7 * np.array([np.cos(theta7), np.sin(theta7)])
        # Trailer 4 Axle
        p8 = p7 - self.L8 * np.array([np.cos(theta8), np.sin(theta8)])
        
        return p0, p0_f, h1, p1, p2, h2, p3, p4, h3, p5, p6, h4, p7, p8
