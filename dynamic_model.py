import numpy as np

class TractorTrailerDynamicModel:
    def __init__(self, 
                 m_v=3000.0, I_v=2600.0, l_f=0.64, l_r=0.64, L_hitch=0.62,
                 m_d=500.0, I_d=500.0, L_bar=1.5,
                 m_t=12000.0, I_t=10400.0, L_trail=0.64, l_rt=0.64,
                 C_f=100000.0, C_r=150000.0, 
                 C_df=150000.0, C_tr=150000.0):
        
        # Tractor Parameters
        self.m_v = m_v
        self.I_v = I_v
        self.l_f = l_f
        self.l_r = l_r
        self.L_hitch = L_hitch
        
        # Drawbar (Dolly) Parameters
        self.m_d = m_d
        self.I_d = I_d
        self.L_bar = L_bar
        
        # Trailer Parameters
        self.m_t = m_t
        self.I_t = I_t
        self.L_trail = L_trail
        self.l_rt = l_rt
        
        # Tire Cornering Stiffnesses
        self.C_f = C_f
        self.C_r = C_r
        self.C_df = C_df   # Drawbar tires (Trailer front tires)
        self.C_tr = C_tr   # Trailer rear tires
        
    def _compute_tire_forces(self, state, steer_angle):
        v_x, v_y, r, r_d, r_t = state['velocities']
        
        theta_v = state['positions'][2]
        theta_d = state['positions'][3]
        theta_t = state['positions'][4]
        
        u_v = v_x * np.cos(theta_v) + v_y * np.sin(theta_v)
        w_v = -v_x * np.sin(theta_v) + v_y * np.cos(theta_v)
        
        eps = 0.1
        u_safe = max(u_v, eps)

        alpha_f = steer_angle - np.arctan2(w_v + self.l_f * r, u_safe)
        alpha_r = -np.arctan2(w_v - self.l_r * r, u_safe)
        
        v_hx = v_x - self.L_hitch * r * np.sin(theta_v)
        v_hy = v_y + self.L_hitch * r * np.cos(theta_v)
        
        v_dx = v_hx - self.L_bar * r_d * np.sin(theta_d)
        v_dy = v_hy + self.L_bar * r_d * np.cos(theta_d)
        
        u_d = v_dx * np.cos(theta_d) + v_dy * np.sin(theta_d)
        w_d = -v_dx * np.sin(theta_d) + v_dy * np.cos(theta_d)
        ud_safe = max(u_d, eps)
        
        alpha_d = -np.arctan2(w_d, ud_safe)
        
        v_tx = v_dx - self.L_trail * r_t * np.sin(theta_t)
        v_ty = v_dy + self.L_trail * r_t * np.cos(theta_t)
        
        u_t = v_tx * np.cos(theta_t) + v_ty * np.sin(theta_t)
        w_t = -v_tx * np.sin(theta_t) + v_ty * np.cos(theta_t)
        ut_safe = max(u_t, eps)
        
        alpha_tr = -np.arctan2(w_t - self.l_rt * r_t, ut_safe)
        
        F_yf = self.C_f * alpha_f
        F_yr = self.C_r * alpha_r
        F_yd = self.C_df * alpha_d
        F_ytr = self.C_tr * alpha_tr
        
        max_F_tractor = 0.8 * 3000 * 9.81
        max_F_trailer = 0.8 * 6000 * 9.81
        
        F_yf = np.clip(F_yf, -max_F_tractor, max_F_tractor)
        F_yr = np.clip(F_yr, -max_F_tractor, max_F_tractor)
        F_yd = np.clip(F_yd, -max_F_trailer, max_F_trailer)
        F_ytr = np.clip(F_ytr, -max_F_trailer, max_F_trailer)
        
        return F_yf, F_yr, F_yd, F_ytr

    def step(self, state, steer_angle, dt=0.01):
        theta_v = state['positions'][2]
        theta_d = state['positions'][3]
        theta_t = state['positions'][4]
        
        v_x, v_y, r, r_d, r_t = state['velocities']
        
        F_yf, F_yr, F_yd, F_ytr = self._compute_tire_forces(state, steer_angle)
        
        # Simple cruise control to maintain forward speed
        u_v = v_x * np.cos(theta_v) + v_y * np.sin(theta_v)
        error = 5.0 - u_v
        F_xr = 3000.0 * error * 5.0  # P-controller
        F_xf = 0.0
        F_xd = 0.0
        F_xtr = 0.0
        
        m_tot = self.m_v + self.m_d + self.m_t
        mdt = self.m_d + self.m_t
        
        M = np.zeros((5, 5))
        
        M[0, 0] = m_tot
        M[0, 1] = 0.0
        M[0, 2] = mdt * self.L_hitch * np.sin(theta_v)
        M[0, 3] = mdt * self.L_bar * np.sin(theta_d)
        M[0, 4] = self.m_t * self.L_trail * np.sin(theta_t)
        
        M[1, 0] = 0.0
        M[1, 1] = m_tot
        M[1, 2] = -mdt * self.L_hitch * np.cos(theta_v)
        M[1, 3] = -mdt * self.L_bar * np.cos(theta_d)
        M[1, 4] = -self.m_t * self.L_trail * np.cos(theta_t)
        
        M[2, 0] = mdt * self.L_hitch * np.sin(theta_v)
        M[2, 1] = -mdt * self.L_hitch * np.cos(theta_v)
        M[2, 2] = self.I_v + mdt * self.L_hitch**2
        M[2, 3] = mdt * self.L_hitch * self.L_bar * np.cos(theta_v - theta_d)
        M[2, 4] = self.m_t * self.L_hitch * self.L_trail * np.cos(theta_v - theta_t)
        
        M[3, 0] = mdt * self.L_bar * np.sin(theta_d)
        M[3, 1] = -mdt * self.L_bar * np.cos(theta_d)
        M[3, 2] = mdt * self.L_hitch * self.L_bar * np.cos(theta_v - theta_d)
        M[3, 3] = self.I_d + mdt * self.L_bar**2
        M[3, 4] = self.m_t * self.L_bar * self.L_trail * np.cos(theta_d - theta_t)
        
        M[4, 0] = self.m_t * self.L_trail * np.sin(theta_t)
        M[4, 1] = -self.m_t * self.L_trail * np.cos(theta_t)
        M[4, 2] = self.m_t * self.L_hitch * self.L_trail * np.cos(theta_v - theta_t)
        M[4, 3] = self.m_t * self.L_bar * self.L_trail * np.cos(theta_d - theta_t)
        M[4, 4] = self.I_t + self.m_t * self.L_trail**2
        
        n = np.zeros(5)
        n[0] = mdt * self.L_hitch * r**2 * np.cos(theta_v) + mdt * self.L_bar * r_d**2 * np.cos(theta_d) + self.m_t * self.L_trail * r_t**2 * np.cos(theta_t)
        n[1] = mdt * self.L_hitch * r**2 * np.sin(theta_v) + mdt * self.L_bar * r_d**2 * np.sin(theta_d) + self.m_t * self.L_trail * r_t**2 * np.sin(theta_t)
        n[2] = -mdt * self.L_hitch * self.L_bar * r_d**2 * np.sin(theta_v - theta_d) - self.m_t * self.L_hitch * self.L_trail * r_t**2 * np.sin(theta_v - theta_t)
        n[3] = -mdt * self.L_hitch * self.L_bar * r**2 * np.sin(theta_v - theta_d) + self.m_t * self.L_bar * self.L_trail * r_t**2 * np.sin(theta_d - theta_t)
        n[4] = -self.m_t * self.L_hitch * self.L_trail * r**2 * np.sin(theta_v - theta_t) - self.m_t * self.L_bar * self.L_trail * r_d**2 * np.sin(theta_d - theta_t)
        
        # Generalized Forces via Virtual Work
        # Global Forces for each axle
        FX_f = F_xf * np.cos(theta_v + steer_angle) - F_yf * np.sin(theta_v + steer_angle)
        FY_f = F_xf * np.sin(theta_v + steer_angle) + F_yf * np.cos(theta_v + steer_angle)
        
        FX_r = F_xr * np.cos(theta_v) - F_yr * np.sin(theta_v)
        FY_r = F_xr * np.sin(theta_v) + F_yr * np.cos(theta_v)
        
        FX_d = F_xd * np.cos(theta_d) - F_yd * np.sin(theta_d)
        FY_d = F_xd * np.sin(theta_d) + F_yd * np.cos(theta_d)
        
        FX_t = F_xtr * np.cos(theta_t) - F_ytr * np.sin(theta_t)
        FY_t = F_xtr * np.sin(theta_t) + F_ytr * np.cos(theta_t)
        
        Q = np.zeros(5)
        # Translation in X and Y
        Q[0] = FX_f + FX_r + FX_d + FX_t
        Q[1] = FY_f + FY_r + FY_d + FY_t
        
        # Rotation of Tractor
        # F_y in tractor frame
        F_yf_v = FY_f * np.cos(theta_v) - FX_f * np.sin(theta_v)
        F_yr_v = FY_r * np.cos(theta_v) - FX_r * np.sin(theta_v)
        F_yd_v = FY_d * np.cos(theta_v) - FX_d * np.sin(theta_v)
        F_yt_v = FY_t * np.cos(theta_v) - FX_t * np.sin(theta_v)
        
        Q[2] = F_yf_v * self.l_f - F_yr_v * self.l_r - F_yd_v * self.L_hitch - F_yt_v * self.L_hitch
        
        # Rotation of Drawbar
        F_yd_d = FY_d * np.cos(theta_d) - FX_d * np.sin(theta_d)
        F_yt_d = FY_t * np.cos(theta_d) - FX_t * np.sin(theta_d)
        Q[3] = -F_yd_d * self.L_bar - F_yt_d * self.L_bar
        
        # Rotation of Trailer
        F_yt_t = FY_t * np.cos(theta_t) - FX_t * np.sin(theta_t)
        Q[4] = -F_yt_t * (self.L_trail + self.l_rt)
        
        try:
            q_ddot = np.linalg.solve(M, Q - n)
        except np.linalg.LinAlgError:
            print("WARNING: Matrix is singular!")
            q_ddot = np.zeros(5)
            
        dv_x, dv_y, dr, dr_d, dr_t = q_ddot
        
        v_x_new = v_x + dv_x * dt
        v_y_new = v_y + dv_y * dt
        r_new = r + dr * dt
        r_d_new = r_d + dr_d * dt
        r_t_new = r_t + dr_t * dt
        
        x_v = state['positions'][0]
        y_v = state['positions'][1]
        x_v_new = x_v + v_x_new * dt
        y_v_new = y_v + v_y_new * dt
        theta_v_new = theta_v + r_new * dt
        theta_d_new = theta_d + r_d_new * dt
        theta_t_new = theta_t + r_t_new * dt
        
        return {
            'positions': np.array([x_v_new, y_v_new, theta_v_new, theta_d_new, theta_t_new]),
            'velocities': np.array([v_x_new, v_y_new, r_new, r_d_new, r_t_new]),
            'accelerations': np.array([dv_x, dv_y, dr, dr_d, dr_t]),
            'hitch_forces': np.array([0.0, 0.0, 0.0, 0.0])
        }
