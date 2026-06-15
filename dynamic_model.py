import numpy as np

class TractorTrailerDynamicModel:
    def __init__(self, 
                 m=5000.0, I_z=10000.0, l_f=2.0, l_r=1.5, d_h=1.5,
                 m_d=500.0, I_zd=1000.0, L_bar=2.0,
                 m_t=10000.0, I_zt=20000.0, l_ft=2.0, l_rt=2.0,
                 C_f=100000.0, C_r=150000.0, 
                 C_df=150000.0, C_tr=150000.0):
        # Tractor Parameters
        self.m = m
        self.I_z = I_z
        self.l_f = l_f
        self.l_r = l_r
        self.d_h = d_h
        
        # Drawbar/Dolly Parameters
        self.m_d = m_d
        self.I_zd = I_zd
        self.L_bar = L_bar
        
        # Trailer Body Parameters
        self.m_t = m_t
        self.I_zt = I_zt
        self.l_ft = l_ft
        self.l_rt = l_rt
        
        # Tire Cornering Stiffnesses
        self.C_f = C_f
        self.C_r = C_r
        self.C_df = C_df   # Drawbar tires (Trailer front tires)
        self.C_tr = C_tr   # Trailer rear tires
        
    def _compute_tire_forces(self, state, steer_angle):
        v_x, v_y, r, v_xd, v_yd, r_d, v_xt, v_yt, r_t = state['velocities']
        
        # Small velocity threshold to prevent singularity in slip angle calculation
        eps = 1e-6
        if abs(v_x) < eps: v_x = np.sign(v_x) * eps if v_x != 0 else eps
        if abs(v_xd) < eps: v_xd = np.sign(v_xd) * eps if v_xd != 0 else eps
        if abs(v_xt) < eps: v_xt = np.sign(v_xt) * eps if v_xt != 0 else eps

        # Tractor Tire Slip Angles
        alpha_f = np.arctan2(v_y + self.l_f * r, v_x) - steer_angle
        alpha_r = np.arctan2(v_y - self.l_r * r, v_x)
        
        # Drawbar Tire Slip Angle (Axle is exactly at CG for drawbar)
        alpha_d = np.arctan2(v_yd, v_xd)
        
        # Trailer Rear Tire Slip Angle
        alpha_tr = np.arctan2(v_yt - self.l_rt * r_t, v_xt)
        
        # Linear Tire Model: F_y = -C * alpha
        F_yf = -self.C_f * alpha_f
        F_yr = -self.C_r * alpha_r
        F_yd = -self.C_df * alpha_d
        F_ytr = -self.C_tr * alpha_tr
        
        # Assuming no longitudinal driving/braking forces for now
        F_xf = 0.0
        F_xr = 0.0
        F_xd = 0.0
        F_xtr = 0.0
        
        return F_xf, F_yf, F_xr, F_yr, F_xd, F_yd, F_xtr, F_ytr

    def step(self, state, steer_angle, dt=0.01):
        """
        Solves the 13x13 matrix system for the 9-DOF formulation
        state: dictionary containing 'positions' and 'velocities'
        """
        # Positions: [x0, y0, theta0, xd, yd, theta1, xt, yt, theta2]
        theta0 = state['positions'][2]
        theta1 = state['positions'][5]
        theta2 = state['positions'][8]
        
        # Velocities: [v_x, v_y, r, v_xd, v_yd, r_d, v_xt, v_yt, r_t]
        v_x, v_y, r, v_xd, v_yd, r_d, v_xt, v_yt, r_t = state['velocities']
        
        delta = steer_angle
        d_theta1 = theta0 - theta1
        d_theta2 = theta1 - theta2
        
        # Compute tire forces
        F_xf, F_yf, F_xr, F_yr, F_xd, F_yd, F_xtr, F_ytr = self._compute_tire_forces(state, steer_angle)
        
        # Initialize A matrix (13x13) and B vector (13x1)
        A = np.zeros((13, 13))
        B = np.zeros(13)
        
        # State vector X = [dv_x, dv_y, dr, dv_xd, dv_yd, dr_d, dv_xt, dv_yt, dr_t, F_hx1, F_hy1, F_hx2, F_hy2]
        # Indices:        0     1     2     3      4      5     6      7      8     9      10     11     12
        
        # --- 1. Tractor Equations of Motion ---
        # Longitudinal: m(dv_x - v_y r) = F_xr + F_xf cos(delta) - F_yf sin(delta) - F_hx1
        A[0, 0] = self.m
        A[0, 9] = 1.0
        B[0] = F_xr + F_xf * np.cos(delta) - F_yf * np.sin(delta) + self.m * v_y * r
        
        # Lateral: m(dv_y + v_x r) = F_yr + F_xf sin(delta) + F_yf cos(delta) - F_hy1
        A[1, 1] = self.m
        A[1, 10] = 1.0
        B[1] = F_yr + F_xf * np.sin(delta) + F_yf * np.cos(delta) - self.m * v_x * r
        
        # Yaw: I_z dr = l_f(F_yf cos(delta) + F_xf sin(delta)) - l_r F_yr - d_h F_hy1
        A[2, 2] = self.I_z
        A[2, 10] = self.d_h
        B[2] = self.l_f * (F_yf * np.cos(delta) + F_xf * np.sin(delta)) - self.l_r * F_yr
        
        # --- 2. Drawbar Equations of Motion ---
        # Longitudinal: m_d(dv_xd - v_yd r_d) = F_xd + F_hx1 cos - F_hx2 cos + F_hy1 sin - F_hy2 sin
        A[3, 3] = self.m_d
        A[3, 9] = -np.cos(d_theta1)
        A[3, 10] = -np.sin(d_theta1)
        A[3, 11] = np.cos(d_theta2)
        A[3, 12] = np.sin(d_theta2)
        B[3] = F_xd + self.m_d * v_yd * r_d
        
        # Lateral: m_d(dv_yd + v_xd r_d) = F_yd - F_hx1 sin + F_hy1 cos + F_hx2 sin - F_hy2 cos
        A[4, 4] = self.m_d
        A[4, 9] = np.sin(d_theta1)
        A[4, 10] = -np.cos(d_theta1)
        A[4, 11] = -np.sin(d_theta2)
        A[4, 12] = np.cos(d_theta2)
        B[4] = F_yd - self.m_d * v_xd * r_d
        
        # Yaw: I_zd dr_d = -L_bar(F_hx1 sin - F_hy1 cos)
        A[5, 5] = self.I_zd
        A[5, 9] = self.L_bar * np.sin(d_theta1)
        A[5, 10] = -self.L_bar * np.cos(d_theta1)
        B[5] = 0.0
        
        # --- 3. Trailer Body Equations of Motion ---
        # Longitudinal: m_t(dv_xt - v_yt r_t) = F_xt + F_hx2
        A[6, 6] = self.m_t
        A[6, 11] = -1.0
        B[6] = F_xtr + self.m_t * v_yt * r_t
        
        # Lateral: m_t(dv_yt + v_xt r_t) = F_yt + F_hy2
        A[7, 7] = self.m_t
        A[7, 12] = -1.0
        B[7] = F_ytr - self.m_t * v_xt * r_t
        
        # Yaw: I_zt dr_t = -l_ft F_hy2 - l_rt F_yt
        A[8, 8] = self.I_zt
        A[8, 12] = self.l_ft
        B[8] = -self.l_rt * F_ytr
        
        # --- 4. Hitch 1 Acceleration Constraints ---
        # X: dv_xd - dv_x cos - dv_y sin + d_h dr sin = ...
        A[9, 3] = 1.0
        A[9, 0] = -np.cos(d_theta1)
        A[9, 1] = -np.sin(d_theta1)
        A[9, 2] = self.d_h * np.sin(d_theta1)
        B[9] = (r - r_d) * (-v_x * np.sin(d_theta1) + (v_y - self.d_h * r) * np.cos(d_theta1))
        
        # Y: dv_yd + L_bar dr_d + dv_x sin - dv_y cos + d_h dr cos = ...
        A[10, 4] = 1.0
        A[10, 5] = self.L_bar
        A[10, 0] = np.sin(d_theta1)
        A[10, 1] = -np.cos(d_theta1)
        A[10, 2] = self.d_h * np.cos(d_theta1)
        B[10] = (r - r_d) * (-v_x * np.cos(d_theta1) - (v_y - self.d_h * r) * np.sin(d_theta1))
        
        # --- 5. Hitch 2 Acceleration Constraints ---
        # X: dv_xt - dv_xd cos - dv_yd sin = ...
        A[11, 6] = 1.0
        A[11, 3] = -np.cos(d_theta2)
        A[11, 4] = -np.sin(d_theta2)
        B[11] = (r_d - r_t) * (-v_xd * np.sin(d_theta2) + v_yd * np.cos(d_theta2))
        
        # Y: dv_yt + l_ft dr_t + dv_xd sin - dv_yd cos = ...
        A[12, 7] = 1.0
        A[12, 8] = self.l_ft
        A[12, 3] = np.sin(d_theta2)
        A[12, 4] = -np.cos(d_theta2)
        B[12] = (r_d - r_t) * (-v_xd * np.cos(d_theta2) - v_yd * np.sin(d_theta2))

        # Solve for accelerations and hitch forces
        try:
            X = np.linalg.solve(A, B)
        except np.linalg.LinAlgError:
            print("WARNING: Matrix is singular!")
            X = np.zeros(13)

        # Extract accelerations
        dv_x, dv_y, dr, dv_xd, dv_yd, dr_d, dv_xt, dv_yt, dr_t = X[0:9]
        
        # Update velocities (Euler integration)
        v_x_new = v_x + dv_x * dt
        v_y_new = v_y + dv_y * dt
        r_new = r + dr * dt
        
        v_xd_new = v_xd + dv_xd * dt
        v_yd_new = v_yd + dv_yd * dt
        r_d_new = r_d + dr_d * dt
        
        v_xt_new = v_xt + dv_xt * dt
        v_yt_new = v_yt + dv_yt * dt
        r_t_new = r_t + dr_t * dt

        # Update positions (Global frame)
        x0, y0 = state['positions'][0:2]
        x0_new = x0 + (v_x_new * np.cos(theta0) - v_y_new * np.sin(theta0)) * dt
        y0_new = y0 + (v_x_new * np.sin(theta0) + v_y_new * np.cos(theta0)) * dt
        theta0_new = theta0 + r_new * dt

        # For dependent bodies, we could integrate their velocities, 
        # but to prevent numerical drift, we strictly enforce the kinematic geometry:
        xd_new = x0_new - self.d_h * np.cos(theta0_new) - self.L_bar * np.cos(theta1 + r_d_new * dt)
        yd_new = y0_new - self.d_h * np.sin(theta0_new) - self.L_bar * np.sin(theta1 + r_d_new * dt)
        theta1_new = theta1 + r_d_new * dt
        
        xt_new = xd_new - self.l_ft * np.cos(theta2 + r_t_new * dt)
        yt_new = yd_new - self.l_ft * np.sin(theta2 + r_t_new * dt)
        theta2_new = theta2 + r_t_new * dt

        return {
            'positions': np.array([x0_new, y0_new, theta0_new, xd_new, yd_new, theta1_new, xt_new, yt_new, theta2_new]),
            'velocities': np.array([v_x_new, v_y_new, r_new, v_xd_new, v_yd_new, r_d_new, v_xt_new, v_yt_new, r_t_new]),
            'accelerations': np.array([dv_x, dv_y, dr, dv_xd, dv_yd, dr_d, dv_xt, dv_yt, dr_t]),
            'hitch_forces': np.array([X[9], X[10], X[11], X[12]]) # Fhx1, Fhy1, Fhx2, Fhy2
        }
