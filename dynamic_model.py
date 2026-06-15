import numpy as np

class TractorTrailerDynamicModel:
    def __init__(self, L0, trailers, dt=0.05, 
                 max_steering_angle=np.radians(30), 
                 max_drawbar_angle=np.radians(30),
                 # Tractor physical parameters
                 m=1500.0, I_z=2500.0, d_h=0.8,
                 l_f=1.2, l_r=1.6,
                 C_f=40000.0, C_r=60000.0,
                 # Trailer physical parameters
                 m_d=200.0, I_zd=100.0, l_fd=0.8, l_rd=0.2, C_d=30000.0,
                 m_t=1000.0, I_zt=1500.0, l_ft=1.0, l_rt=1.5, C_t=40000.0):
        """
        Initialize the dynamic model parameters for Tractor + 1 Drawbar Trailer (Dolly + Trailer Body).
        """
        self.L0 = L0
        self.trailers = trailers
        self.dt = dt
        self.max_steering_angle = max_steering_angle
        self.max_drawbar_angle = max_drawbar_angle
        
        # Tractor parameters
        self.m = m
        self.I_z = I_z
        self.d_h = d_h
        self.l_f = l_f
        self.l_r = l_r
        self.C_f = C_f
        self.C_r = C_r
        
        # Dolly parameters
        self.m_d = m_d
        self.I_zd = I_zd
        self.l_fd = l_fd
        self.l_rd = l_rd
        self.C_d = C_d
        
        # Trailer body parameters
        self.m_t = m_t
        self.I_zt = I_zt
        self.l_ft = l_ft
        self.l_rt = l_rt
        self.C_t = C_t

        # Drawbar and trailer geometry from config
        self.L1 = trailers[0]['L_bar']
        self.L2 = trailers[0]['L_trl']

    def get_state_derivative(self, state, Fxr, delta):
        """
        Calculate the time derivative of the state vector:
        state = [x0, y0, theta0, theta1, theta2, vx, vy, r, rd, rt]
        
        Args:
            state (np.array): Current state vector
            Fxr (float): Rear wheel longitudinal force (driving force)
            delta (float): Front wheel steering angle (rad)
            
        Returns:
            np.array: State derivative [dx0, dy0, dtheta0, dtheta1, dtheta2, dvx, dvy, dr, drd, drt]
        """
        # Unpack state
        x0, y0, theta0, theta1, theta2, vx, vy, r, rd, rt = state
        
        # Relative angles
        delta_theta1 = theta0 - theta1
        delta_theta2 = theta1 - theta2
        
        # Dolly velocities in Dolly body frame (derived from joint velocity constraints)
        vxd = vx * np.cos(delta_theta1) + (vy - self.d_h * r) * np.sin(delta_theta1)
        vyd = -vx * np.sin(delta_theta1) + (vy - self.d_h * r) * np.cos(delta_theta1) - self.l_fd * rd
        
        # Trailer body velocities in Trailer body frame
        vxt = vxd * np.cos(delta_theta2) + (vyd - self.l_rd * rd) * np.sin(delta_theta2)
        vyt = -vxd * np.sin(delta_theta2) + (vyd - self.l_rd * rd) * np.cos(delta_theta2) - self.l_ft * rt
        
        # Avoid division by zero at low speeds by regularizing vx
        vx_reg = np.max([vx, 0.5])
        vxd_reg = np.max([vxd, 0.5])
        vxt_reg = np.max([vxt, 0.5])
        
        # Tire slip angles
        alpha_f = np.arctan2(vy + self.l_f * r, vx_reg) - delta
        alpha_r = np.arctan2(vy - self.l_r * r, vx_reg)
        alpha_d = np.arctan2(vyd, vxd_reg)
        alpha_t = np.arctan2(vyt - self.l_rt * rt, vxt_reg)
        
        # Tire forces
        Fyf = -self.C_f * alpha_f
        Fyr = -self.C_r * alpha_r
        Fyd = -self.C_d * alpha_d
        Fyt = -self.C_t * alpha_t
        
        # Longitudinal forces (other than rear driving force Fxr are zero)
        Fxf = 0.0
        Fxd = 0.0
        Fxt = 0.0
        
        # Set up linear system A * X = b
        # X = [dvx, dvy, dr, dvxd, dvyd, drd, dvxt, dvyt, drt, Fhx1, Fhy1, Fhx2, Fhy2] (13 unknowns)
        A = np.zeros((13, 13))
        b = np.zeros(13)
        
        c1 = np.cos(delta_theta1)
        s1 = np.sin(delta_theta1)
        c2 = np.cos(delta_theta2)
        s2 = np.sin(delta_theta2)
        
        # 1. Tractor Longitudinal
        A[0, 0] = self.m
        A[0, 9] = 1.0
        b[0] = Fxr + Fxf * np.cos(delta) - Fyf * np.sin(delta) + self.m * vy * r
        
        # 2. Tractor Lateral
        A[1, 1] = self.m
        A[1, 10] = 1.0
        b[1] = Fyr + Fxf * np.sin(delta) + Fyf * np.cos(delta) - self.m * vx * r
        
        # 3. Tractor Yaw
        A[2, 2] = self.I_z
        A[2, 10] = self.d_h
        b[2] = self.l_f * (Fyf * np.cos(delta) + Fxf * np.sin(delta)) - self.l_r * Fyr
        
        # 4. Dolly Longitudinal
        A[3, 3] = self.m_d
        A[3, 9] = -c1
        A[3, 10] = -s1
        A[3, 11] = c2
        A[3, 12] = s2
        b[3] = Fxd + self.m_d * vyd * rd
        
        # 5. Dolly Lateral
        A[4, 4] = self.m_d
        A[4, 9] = s1
        A[4, 10] = -c1
        A[4, 11] = -s2
        A[4, 12] = c2
        b[4] = Fyd - self.m_d * vxd * rd
        
        # 6. Dolly Yaw
        A[5, 5] = self.I_zd
        A[5, 9] = self.l_fd * s1
        A[5, 10] = -self.l_fd * c1
        A[5, 11] = -self.l_rd * s2
        A[5, 12] = self.l_rd * c2
        b[5] = 0.0
        
        # 7. Trailer Body Longitudinal
        A[6, 6] = self.m_t
        A[6, 11] = -1.0
        b[6] = Fxt + self.m_t * vyt * rt
        
        # 8. Trailer Body Lateral
        A[7, 7] = self.m_t
        A[7, 12] = -1.0
        b[7] = Fyt - self.m_t * vxt * rt
        
        # 9. Trailer Body Yaw
        A[8, 8] = self.I_zt
        A[8, 12] = self.l_ft
        b[8] = -self.l_rt * Fyt
        
        # 10. Constraint H1 X
        A[9, 0] = -c1
        A[9, 1] = -s1
        A[9, 2] = self.d_h * s1
        A[9, 3] = 1.0
        b[9] = (r - rd) * (-vx * s1 + (vy - self.d_h * r) * c1)
        
        # 11. Constraint H1 Y
        A[10, 0] = s1
        A[10, 1] = -c1
        A[10, 2] = self.d_h * c1
        A[10, 4] = 1.0
        A[10, 5] = self.l_fd
        b[10] = (r - rd) * (-vx * c1 - (vy - self.d_h * r) * s1)
        
        # 12. Constraint H2 X
        A[11, 3] = -c2
        A[11, 4] = -s2
        A[11, 5] = self.l_rd * s2
        A[11, 6] = 1.0
        b[11] = (rd - rt) * (-vxd * s2 + (vyd - self.l_rd * rd) * c2)
        
        # 13. Constraint H2 Y
        A[12, 3] = s2
        A[12, 4] = -c2
        A[12, 5] = self.l_rd * c2
        A[12, 7] = 1.0
        A[12, 8] = self.l_ft
        b[12] = (rd - rt) * (-vxd * c2 - (vyd - self.l_rd * rd) * s2)
        
        # Solve the system
        try:
            X = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            # Fallback if singular (should not happen for well-posed physical models)
            X = np.zeros(13)
            
        # Extract derivatives
        dvx, dvy, dr = X[0], X[1], X[2]
        drd, drt = X[5], X[8]
        
        # State derivatives:
        # [dx0, dy0, dtheta0, dtheta1, dtheta2, dvx, dvy, dr, drd, drt]
        dx0 = vx * np.cos(theta0) - vy * np.sin(theta0)
        dy0 = vx * np.sin(theta0) + vy * np.cos(theta0)
        dtheta0 = r
        dtheta1 = rd
        dtheta2 = rt
        
        return np.array([dx0, dy0, dtheta0, dtheta1, dtheta2, dvx, dvy, dr, drd, drt])

    def update(self, state, Fxr, delta):
        """
        Update the state using sub-stepped Runge-Kutta 4th order (RK4) integration for numerical stability.
        """
        # Clamp Steering Angle
        delta = np.clip(delta, -self.max_steering_angle, self.max_steering_angle)
        
        curr_state = np.copy(state)
        
        n_substeps = 20  # Use 20 sub-steps for very high stability
        dt_sub = self.dt / n_substeps
        
        for _ in range(n_substeps):
            k1 = self.get_state_derivative(curr_state, Fxr, delta)
            k2 = self.get_state_derivative(curr_state + 0.5 * dt_sub * k1, Fxr, delta)
            k3 = self.get_state_derivative(curr_state + 0.5 * dt_sub * k2, Fxr, delta)
            k4 = self.get_state_derivative(curr_state + dt_sub * k3, Fxr, delta)
            
            curr_state = curr_state + (dt_sub / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            
            # Normalize angles to [-pi, pi] after each sub-step
            for idx in [2, 3, 4]:
                curr_state[idx] = (curr_state[idx] + np.pi) % (2 * np.pi) - np.pi
                
        return curr_state

    def get_coordinates(self, state):
        """
        Calculate coordinates of all key points for visualization.
        Returns: [p0, p0_f, h1, p1, p2]
        """
        x0, y0, theta0, theta1, theta2 = state[0], state[1], state[2], state[3], state[4]
        
        # Tractor Rear Axle
        p0 = np.array([x0, y0])
        
        # Tractor Front Axle
        p0_f = p0 + self.L0 * np.array([np.cos(theta0), np.sin(theta0)])
        
        # First Hitch (rear of tractor)
        h1 = p0 - self.d_h * np.array([np.cos(theta0), np.sin(theta0)])
        
        # Dolly position (rear axle center of drawbar dolly)
        p1 = h1 - self.L1 * np.array([np.cos(theta1), np.sin(theta1)])
        
        # Trailer axle (rear of trailer body)
        p2 = p1 - self.L2 * np.array([np.cos(theta2), np.sin(theta2)])
        
        return [p0, p0_f, h1, p1, p2]
