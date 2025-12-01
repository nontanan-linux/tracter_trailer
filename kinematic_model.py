import numpy as np

class TractorTrailerModel:
    def __init__(self, L0, dh, L1, L2, dt=0.1, max_steering_angle=np.radians(30), max_drawbar_angle=np.radians(30), max_velocity=5.0):
        """
        Initialize the kinematic model parameters for Tractor + Dolly + Trailer.
        
        Args:
            L0 (float): Wheelbase of the tractor.
            dh (float): Distance from tractor rear axle to hitch.
            L1 (float): Drawbar length (Hitch to Dolly Axle).
            L2 (float): Trailer length (Dolly Axle to Trailer Axle).
            dt (float): Time step for discrete update.
            max_steering_angle (float): Maximum steering angle for tractor (radians).
            max_drawbar_angle (float): Maximum relative angle for drawbar (radians).
            max_velocity (float): Maximum longitudinal velocity (m/s).
        """
        self.L0 = L0
        self.dh = dh
        self.L1 = L1
        self.L2 = L2
        self.dt = dt
        self.max_steering_angle = max_steering_angle
        self.max_drawbar_angle = max_drawbar_angle
        self.max_velocity = max_velocity

    def get_state_derivative(self, state, v0, delta):
        """
        Calculate the time derivative of the state.
        
        Args:
            state (list or np.array): [x0, y0, theta0, theta1, theta2]
            v0 (float): Longitudinal velocity of the tractor.
            delta (float): Steering angle of the tractor.
            
        Returns:
            np.array: Time derivative of the state [dx0, dy0, dtheta0, dtheta1, dtheta2]
        """
        x0, y0, theta0, theta1, theta2 = state
        
        # Tractor Kinematics
        dx0 = v0 * np.cos(theta0)
        dy0 = v0 * np.sin(theta0)
        dtheta0 = (v0 / self.L0) * np.tan(delta)
        
        # Hitch Velocity components (in world frame)
        # Hitch position H = P0 - dh * [cos(theta0), sin(theta0)]
        # v_h = v0 - dh * dtheta0 * [-sin(theta0), cos(theta0)] (This is wrong vector algebra)
        # Let's do it in body frame components or direct differentiation.
        # v_h_x = v0 cos(theta0) + dh * sin(theta0) * dtheta0
        # v_h_y = v0 sin(theta0) - dh * cos(theta0) * dtheta0
        # Actually, simpler:
        # Velocity of hitch point along the drawbar direction (theta1) drives the dolly.
        # v_h_long = v0 * cos(theta0 - theta1) + dh * dtheta0 * sin(theta0 - theta1)
        # v_h_lat  = -v0 * sin(theta0 - theta1) + dh * dtheta0 * cos(theta0 - theta1)
        
        # Dolly (Drawbar) Kinematics
        # The hitch pulls the drawbar. 
        # The velocity of the hitch projected onto the drawbar normal drives theta1.
        # dtheta1 = (1/L1) * (Velocity of hitch perpendicular to drawbar)
        # Perpendicular component: v0 * sin(theta0 - theta1) + dh * dtheta0 * cos(theta0 - theta1)
        # Wait, sign convention.
        # If hitch moves left relative to drawbar, drawbar rotates CCW (+).
        # Let's use the standard formula:
        # dtheta1 = (1/L1) * ( v0 * sin(theta0 - theta1) - dh * dtheta0 * cos(theta0 - theta1) )
        # Let's verify signs.
        # If theta0 = 0, theta1 = 0. Hitch moves forward. No rotation. Correct.
        # If theta0 = 90, theta1 = 0. Hitch moves up (y+). Relative to drawbar (x+), this is +90 deg.
        # sin(90) = 1. dtheta1 > 0. Correct.
        
        term1 = v0 * np.sin(theta0 - theta1)
        term2 = self.dh * dtheta0 * np.cos(theta0 - theta1)
        dtheta1 = (1 / self.L1) * (term1 - term2)
        
        # Dolly Axle Velocity
        # Velocity of dolly axle (P1) along the trailer direction (theta2).
        # v1 = v0 * cos(theta0 - theta1) + dh * dtheta0 * sin(theta0 - theta1)
        # This is the velocity of the hitch projected along the drawbar.
        # The dolly axle velocity is this v1.
        
        v1 = v0 * np.cos(theta0 - theta1) + self.dh * dtheta0 * np.sin(theta0 - theta1)
        
        # Trailer Kinematics
        # The dolly axle pulls the trailer.
        # dtheta2 = (v1 / L2) * sin(theta1 - theta2)
        dtheta2 = (v1 / self.L2) * np.sin(theta1 - theta2)
        
        return np.array([dx0, dy0, dtheta0, dtheta1, dtheta2])

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
        
        # Clamp Drawbar Angle (relative to Tractor)
        # theta1 should be within [theta0 - max, theta0 + max]
        theta0_new = new_state[2]
        theta1_new = new_state[3]
        
        # Normalize relative angle to [-pi, pi]
        rel_angle = theta1_new - theta0_new
        while rel_angle > np.pi: rel_angle -= 2*np.pi
        while rel_angle < -np.pi: rel_angle += 2*np.pi
        
        if rel_angle > self.max_drawbar_angle:
            theta1_new = theta0_new + self.max_drawbar_angle
        elif rel_angle < -self.max_drawbar_angle:
            theta1_new = theta0_new - self.max_drawbar_angle
            
        new_state[3] = theta1_new
        
        return new_state
        


    def get_coordinates(self, state):
        """
        Calculate coordinates of all key points for visualization.
        """
        x0, y0, theta0, theta1, theta2 = state
        # Tractor Rear Axle
        p0 = np.array([x0, y0])
        # Tractor Front Axle
        p0_f = p0 + self.L0 * np.array([np.cos(theta0), np.sin(theta0)])
        # Hitch
        h = p0 - self.dh * np.array([np.cos(theta0), np.sin(theta0)])
        # Dolly Axle
        p1 = h - self.L1 * np.array([np.cos(theta1), np.sin(theta1)])
        # Trailer Axle
        p2 = p1 - self.L2 * np.array([np.cos(theta2), np.sin(theta2)])
        return p0, p0_f, h, p1, p2
