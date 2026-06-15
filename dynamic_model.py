import numpy as np

class TractorTrailerDynamicModel:
    def __init__(self, 
                 m=5000.0, I_z=10000.0, l_f=2.0, l_r=1.5, d_h=1.5,
                 m_t=1000.0, I_zt=2000.0, l_tf=1.5, l_tr=1.5,
                 C_f=100000.0, C_r=150000.0, 
                 C_tf=150000.0, C_tr=150000.0,
                 L_bar=2.0):
        # Tractor Parameters
        self.m = m
        self.I_z = I_z
        self.l_f = l_f
        self.l_r = l_r
        self.d_h = d_h
        
        # Trailer Parameters (Full Trailer as a single body)
        self.m_t = m_t
        self.I_zt = I_zt
        self.l_tf = l_tf
        self.l_tr = l_tr
        
        # Cornering Stiffness
        self.C_f = C_f
        self.C_r = C_r
        self.C_tf = C_tf  # Stiffness of steerable front axle
        self.C_tr = C_tr  # Stiffness of fixed rear axle
        
        # Drawbar length
        self.L_bar = L_bar

    def get_state_derivative(self, t, state, v0, delta):
        '''
        State vector:
        [0] x0: Tractor X position
        [1] y0: Tractor Y position
        [2] theta0: Tractor Yaw angle
        [3] vx: Tractor longitudinal velocity
        [4] vy: Tractor lateral velocity
        [5] r: Tractor yaw rate
        [6] xt: Trailer X position
        [7] yt: Trailer Y position
        [8] thetat: Trailer Yaw angle
        [9] vxt: Trailer longitudinal velocity
        [10] vyt: Trailer lateral velocity
        [11] rt: Trailer yaw rate
        [12] thetad: Drawbar angle
        '''
        x0, y0, theta0, vx, vy, r, xt, yt, thetat, vxt, vyt, rt, thetad = state
        
        # Regularization for low speeds
        vx_reg = max(abs(vx), 0.1) * np.sign(vx) if vx != 0 else 0.1
        vxt_reg = max(abs(vxt), 0.1) * np.sign(vxt) if vxt != 0 else 0.1

        # Calculate slip angles
        alpha_f = np.arctan2(vy + self.l_f * r, vx_reg) - delta
        alpha_r = np.arctan2(vy - self.l_r * r, vx_reg)
        
        # Tractor lateral tire forces (Linear model)
        Fyf = -self.C_f * alpha_f
        Fyr = -self.C_r * alpha_r
        
        # Trailer front axle is attached via drawbar. The steering angle of the front wheels is the drawbar angle relative to the trailer body.
        delta_trailer = thetad - thetat
        
        # Calculate trailer slip angles
        alpha_tf = np.arctan2(vyt + self.l_tf * rt, vxt_reg) - delta_trailer
        alpha_tr = np.arctan2(vyt - self.l_tr * rt, vxt_reg)
        
        # Trailer lateral tire forces
        Fyt_front = -self.C_tf * alpha_tf
        Fyt_rear = -self.C_tr * alpha_tr
        
        # Tractor longitudinal forces (Simple speed controller)
        Fxf = 1000.0 * (v0 - vx)
        Fxr = Fxf
        
        # Drag
        F_drag_x = 0.5 * vx * abs(vx)
        F_drag_t = 0.5 * vxt * abs(vxt)

        # Precompute trigonometric terms
        c1 = np.cos(theta0 - thetad)
        s1 = np.sin(theta0 - thetad)
        c2 = np.cos(thetat - thetad)
        s2 = np.sin(thetat - thetad)
        
        cd0 = np.cos(thetad - theta0)
        sd0 = np.sin(thetad - theta0)
        cdt = np.cos(thetad - thetat)
        sdt = np.sin(thetad - thetat)

        # Assemble the 8x8 matrix A
        A = np.zeros((8, 8))
        b = np.zeros(8)
        
        # Unknowns vector x = [dot_vx, dot_vy, dot_r, dot_vxt, dot_vyt, dot_rt, dot_rd, Fd]^T
        
        # 1. Tractor Longitudinal
        A[0, 0] = self.m
        A[0, 7] = -cd0
        b[0] = Fxr + Fxf * np.cos(delta) - Fyf * np.sin(delta) + self.m * vy * r - F_drag_x
        
        # 2. Tractor Lateral
        A[1, 1] = self.m
        A[1, 7] = -sd0
        b[1] = Fyf * np.cos(delta) + Fxf * np.sin(delta) + Fyr - self.m * vx * r
        
        # 3. Tractor Yaw
        A[2, 2] = self.I_z
        A[2, 7] = self.d_h * sd0
        b[2] = self.l_f * (Fyf * np.cos(delta) + Fxf * np.sin(delta)) - self.l_r * Fyr
        
        # 4. Trailer Longitudinal
        A[3, 3] = self.m_t
        A[3, 7] = cdt
        b[3] = Fyt_front * sdt + self.m_t * vyt * rt - F_drag_t
        
        # 5. Trailer Lateral
        A[4, 4] = self.m_t
        A[4, 7] = sdt
        b[4] = -Fyt_front * cdt + Fyt_rear - self.m_t * vxt * rt
        
        # 6. Trailer Yaw
        A[5, 5] = self.I_zt
        A[5, 7] = self.l_tf * sdt
        b[5] = -self.l_tf * Fyt_front * cdt - self.l_tr * Fyt_rear
        
        # 7. Hitch X-Acceleration Constraint
        A[6, 0] = c1
        A[6, 1] = -s1
        A[6, 2] = self.d_h * s1
        A[6, 3] = -c2
        A[6, 4] = s2
        A[6, 5] = self.l_tf * s2
        A[6, 6] = self.L_bar
        b[6] = vx * s1 * r + (vy - self.d_h * r) * c1 * r - vxt * s2 * rt - (vyt + self.l_tf * rt) * c2 * rt
        
        # 8. Hitch Y-Acceleration Constraint
        A[7, 0] = s1
        A[7, 1] = c1
        A[7, 2] = -self.d_h * c1
        A[7, 3] = -s2
        A[7, 4] = -c2
        A[7, 5] = -self.l_tf * c2
        
        # Note: we need to find the Drawbar angular velocity rd to compute the constraint
        # rd can be found by solving the kinematic velocity constraint
        # vx * s1 + (vy - d_h*r) * c1 - vxt * s2 - (vyt + l_tf*rt) * c2 - L_bar * rd = 0
        rd = (vx * s1 + (vy - self.d_h * r) * c1 - vxt * s2 - (vyt + self.l_tf * rt) * c2) / self.L_bar
        
        A[7, 6] = -self.L_bar
        b[7] = -vx * c1 * r + (vy - self.d_h * r) * s1 * r + vxt * c2 * rt - (vyt + self.l_tf * rt) * s2 * rt
        
        # Solve the 8x8 system
        try:
            res = np.linalg.solve(A, b)
            dot_vx, dot_vy, dot_r, dot_vxt, dot_vyt, dot_rt, dot_rd, Fd = res
        except np.linalg.LinAlgError:
            dot_vx, dot_vy, dot_r, dot_vxt, dot_vyt, dot_rt, dot_rd, Fd = [0.0]*8
            
        # Kinematics in global frame
        dot_x0 = vx * np.cos(theta0) - vy * np.sin(theta0)
        dot_y0 = vx * np.sin(theta0) + vy * np.cos(theta0)
        dot_theta0 = r
        
        dot_xt = vxt * np.cos(thetat) - vyt * np.sin(thetat)
        dot_yt = vxt * np.sin(thetat) + vyt * np.cos(thetat)
        dot_thetat = rt
        
        dot_thetad = dot_rd

        return np.array([
            dot_x0, dot_y0, dot_theta0,
            dot_vx, dot_vy, dot_r,
            dot_xt, dot_yt, dot_thetat,
            dot_vxt, dot_vyt, dot_rt,
            dot_thetad
        ])

    def update(self, state, v0, delta, dt=0.05):
        # RK4 Integrator
        k1 = self.get_state_derivative(0, state, v0, delta)
        k2 = self.get_state_derivative(0, state + 0.5 * dt * k1, v0, delta)
        k3 = self.get_state_derivative(0, state + 0.5 * dt * k2, v0, delta)
        k4 = self.get_state_derivative(0, state + dt * k3, v0, delta)
        
        new_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        
        # Normalize angles
        new_state[2] = (new_state[2] + np.pi) % (2 * np.pi) - np.pi
        new_state[8] = (new_state[8] + np.pi) % (2 * np.pi) - np.pi
        new_state[12] = (new_state[12] + np.pi) % (2 * np.pi) - np.pi
        
        return new_state

    def get_coordinates(self, state):
        x0, y0, theta0, vx, vy, r, xt, yt, thetat, vxt, vyt, rt, thetad = state
        
        p0 = np.array([x0, y0])
        p0_f = p0 + self.l_f * np.array([np.cos(theta0), np.sin(theta0)])
        h1 = p0 - self.d_h * np.array([np.cos(theta0), np.sin(theta0)])
        
        # Drawbar connects H1 to front axle of trailer
        p_axle_f = h1 - self.L_bar * np.array([np.cos(thetad), np.sin(thetad)])
        
        # Trailer body is connected at the front axle
        p_axle_r = p_axle_f - (self.l_tf + self.l_tr) * np.array([np.cos(thetat), np.sin(thetat)])
        
        return [p0, p0_f, h1, p_axle_f, p_axle_r]
