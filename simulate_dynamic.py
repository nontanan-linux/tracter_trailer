import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from dynamic_model import TractorTrailerDynamicModel

class TractorTrailerSimulator:
    def __init__(self, T=20.0, dt=0.05, v_target=2.0):
        self.T = T
        self.dt = dt
        self.v_target = v_target
        
        # --- Parameters ---
        self.SAVE_ANIMATION = False
        self.PLOT_DRAWBAR_TRAJECTORY = True
        
        # Overall visual dimensions for the plots
        self.tractor_len = 2.95
        self.tractor_width = 1.30
        self.tractor_overhang = 0.62
        self.L0 = 1.28
        
        self.trailer_body_len = 4.0
        self.trailer_width = 1.30
        self.W = 1.09        # Track Width
        
        # Trailer Configuration
        self.trailers = [
            {'L_bar': 1.5, 'L_trl': 3.5, 'dh_prev': 0.62},
        ]
        self.num_trailers = len(self.trailers)
        self.wheel_width = 0.2
        self.wheel_diam = 0.8
        self.trailer_overhang = 0.5
        
        self.model = TractorTrailerDynamicModel(
            L0=self.L0,
            trailers=self.trailers,
            dt=self.dt
        )
        
        self.states = []
        self.inputs = []
        self.velocities = []
        self.trajectory = []
        self.drawbar_trajectories = []
        
    def run_simulation(self):
        print("Running dynamic simulation...")
        
        # Initial State: [x0, y0, theta0, theta1, theta2, vx, vy, r, rd, rt]
        state = np.zeros(10)
        state[5] = 2.0  # Initial forward velocity
        
        # PI Controller for speed
        Kp = 1000.0
        Ki = 100.0
        v_error_integral = 0.0
        
        steps = int(self.T / self.dt)
        
        for i in range(steps):
            t = i * self.dt
            
            # Speed Control
            vx = state[5]
            v_error = self.v_target - vx
            v_error_integral += v_error * self.dt
            Fxr = Kp * v_error + Ki * v_error_integral
            Fxr = np.clip(Fxr, -10000.0, 10000.0)
            
            # Steering input
            delta = np.radians(8) * (1 - np.exp(-t))
            
            self.states.append(state)
            self.inputs.append([Fxr, delta])
            
            theta0, theta1, theta2 = state[2], state[3], state[4]
            r, rd, rt = state[7], state[8], state[9]
            vy = state[6]
            
            dx0 = vx * np.cos(theta0) - vy * np.sin(theta0)
            dy0 = vx * np.sin(theta0) + vy * np.cos(theta0)
            
            # Dolly and Trailer velocities
            dxd = dx0 + self.model.d_h * r * np.sin(theta0) + self.model.l_fd * rd * np.sin(theta1)
            dyd = dy0 - self.model.d_h * r * np.cos(theta0) - self.model.l_fd * rd * np.cos(theta1)
            
            dxt = dx0 + self.model.d_h * r * np.sin(theta0) + (self.model.l_fd + self.model.l_rd) * rd * np.sin(theta1) + self.model.l_ft * rt * np.sin(theta2)
            dyt = dy0 - self.model.d_h * r * np.cos(theta0) - (self.model.l_fd + self.model.l_rd) * rd * np.cos(theta1) - self.model.l_ft * rt * np.cos(theta2)
            
            vxt = dxt * np.cos(theta2) + dyt * np.sin(theta2)
            vyt = -dxt * np.sin(theta2) + dyt * np.cos(theta2)
            
            T_total = 0.5 * self.model.m_t * (vxt**2 + vyt**2)
            self.velocities.append([vxt, T_total])
            
            coords = self.model.get_coordinates(state)
            self.trajectory.append(coords[0])
            
            if i % 10 == 0:
                x0_pos, y0_pos = coords[0][0], coords[0][1]
                xt_pos, yt_pos = coords[4][0], coords[4][1]
                v0_mag = np.sqrt(vx**2 + vy**2)
                vt_mag = np.sqrt(vxt**2 + vyt**2)
                print(f"t={t:05.2f}s | Tractor: x={x0_pos:06.2f}, y={y0_pos:06.2f}, v={v0_mag:05.2f} m/s | Trailer: x={xt_pos:06.2f}, y={yt_pos:06.2f}, v={vt_mag:05.2f} m/s")
                sys.stdout.flush()
                
            current_drawbars = []
            for k in range(self.num_trailers):
                idx_dolly = 3 + 3*k
                current_drawbars.append(coords[idx_dolly])
            self.drawbar_trajectories.append(current_drawbars)
            
            state = self.model.update(state, Fxr, delta)
            
        self.states = np.array(self.states)
        self.inputs = np.array(self.inputs)
        self.trajectory = np.array(self.trajectory)
        self.drawbar_trajectories = np.array(self.drawbar_trajectories)
        print("Simulation complete. Preparing animation...")

    @staticmethod
    def draw_box(ax, center, length, width, angle, color='gray', alpha=0.5):
        c, s = np.cos(angle), np.sin(angle)
        dx, dy = -length/2, -width/2
        rx = dx*c - dy*s
        ry = dx*s + dy*c
        rect = Rectangle((center[0]+rx, center[1]+ry), length, width, angle=np.degrees(angle), color=color, alpha=alpha, ec='black')
        ax.add_patch(rect)
        return rect

    def draw_wheels_at_axle(self, ax, center, angle, track_width, steered_angle=0, color='black'):
        wl_pos = center + (track_width/2) * np.array([-np.sin(angle), np.cos(angle)])
        wr_pos = center - (track_width/2) * np.array([-np.sin(angle), np.cos(angle)])
        
        wa = angle + steered_angle
        
        w_objs = []
        for w_pos in [wl_pos, wr_pos]:
            w = self.draw_box(ax, w_pos, self.wheel_diam, self.wheel_width, wa, color=color, alpha=1.0)
            w_objs.append(w)
        
        l, = ax.plot([wl_pos[0], wr_pos[0]], [wl_pos[1], wr_pos[1]], 'k-', lw=2)
        return w_objs + [l]

    def update_plot(self, i, ax, trace, status_texts, drawbar_traces, patches_list):
        for p in patches_list:
            p.remove()
        patches_list.clear()
        
        trace.set_data(self.trajectory[:i, 0], self.trajectory[:i, 1])
        
        if self.PLOT_DRAWBAR_TRAJECTORY:
            drawbar_traces[0].set_data(self.drawbar_trajectories[:i, 0, 0], self.drawbar_trajectories[:i, 0, 1])
            
        state = self.states[i]
        Fxr_curr = self.inputs[i, 0]
        delta_curr = self.inputs[i, 1]
        vels = self.velocities[i]
        
        vx_curr = state[5]
        status_texts[0].set_text(f'Tractor Vx: {vx_curr:.2f} m/s\nForce Fxr: {Fxr_curr:.1f} N\nSteer: {np.degrees(delta_curr):.1f} deg')
        
        psi = state[2] - state[3]
        psi = (psi + np.pi) % (2 * np.pi) - np.pi
        
        T_total_display = vels[1]
        status_texts[1].set_text(f'Trailer Vxt: {vels[0]:.2f} m/s\nDrawbar Ang: {np.degrees(psi):.1f} deg\nKinetic Energy (T): {T_total_display/1000:.1f} kJ')
        
        coords = self.model.get_coordinates(state)
        p0 = coords[0]
        p0_f = coords[1]
        theta0 = state[2]
        
        p_tractor_c = p0 + ((self.tractor_len / 2) - self.tractor_overhang) * np.array([np.cos(theta0), np.sin(theta0)])
        patches_list.append(self.draw_box(ax, p_tractor_c, self.tractor_len, self.tractor_width, theta0, color='orangered', alpha=0.5))
        
        patches_list.extend(self.draw_wheels_at_axle(ax, p0, theta0, self.W)) 
        patches_list.extend(self.draw_wheels_at_axle(ax, p0_f, theta0, self.W, steered_angle=delta_curr)) 
        
        p_tr_rear_face = p0 - self.tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        h1 = coords[2]
        l_tr_tail, = ax.plot([p_tr_rear_face[0], h1[0]], [p_tr_rear_face[1], h1[1]], 'k-', lw=2)
        patches_list.append(l_tr_tail)
        
        h_curr = coords[2]
        p_dolly = coords[3]
        p_axle = coords[4]
        
        theta_drawbar = state[3]
        theta_trailer = state[4]
        
        l_db, = ax.plot([h_curr[0], p_dolly[0]], [h_curr[1], p_dolly[1]], 'k-', lw=3)
        patches_list.append(l_db)
        
        p_trailer_c = p_dolly - 2.0 * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
        patches_list.append(self.draw_box(ax, p_trailer_c, self.trailer_body_len, self.trailer_width, theta_trailer, color='blue', alpha=0.5))
        
        patches_list.extend(self.draw_wheels_at_axle(ax, p_dolly, theta_drawbar, self.W)) 
        patches_list.extend(self.draw_wheels_at_axle(ax, p_axle, theta_trailer, self.W))  
        
        pt_h, = ax.plot(h_curr[0], h_curr[1], 'ko', ms=5)
        patches_list.append(pt_h)
        
        p_tl_rear_face = p_axle - self.trailer_overhang * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
        p_stub = p_tl_rear_face - 0.1 * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
        l_tail, = ax.plot([p_tl_rear_face[0], p_stub[0]], [p_tl_rear_face[1], p_stub[1]], 'k-', lw=2)
        patches_list.append(l_tail)

        return patches_list + [trace] + status_texts + drawbar_traces

    def animate(self):
        fig, ax = plt.subplots(figsize=(12, 12))
        ax.set_aspect('equal')
        ax.set_xlim(-10, 30)
        ax.set_ylim(-10, 40)
        ax.grid(True)
        
        ax.set_title(f"Tractor-Trailer Dynamic Simulation ({self.num_trailers} Trailer)")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        
        status_texts = []
        t_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=10, color='blue',
                         verticalalignment='top', fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6, edgecolor='none'))
        status_texts.append(t_text)
        
        cmap = plt.get_cmap('jet')
        trailer_colors = [cmap(float(k) / self.num_trailers) for k in range(self.num_trailers)]
        
        tr_text = ax.text(0.05, 0.90, '', transform=ax.transAxes, fontsize=10, color=trailer_colors[0],
                          verticalalignment='top', fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6, edgecolor='none'))
        status_texts.append(tr_text)
        
        trace, = ax.plot([], [], 'b--', alpha=0.5, label='Tractor Path')
        
        drawbar_traces = []
        if self.PLOT_DRAWBAR_TRAJECTORY:
            d_trace, = ax.plot([], [], '--', color=trailer_colors[0], alpha=0.4, linewidth=1)
            drawbar_traces.append(d_trace)
            
        patches_list = []
        
        ani = animation.FuncAnimation(fig, self.update_plot, frames=len(self.states), 
                                      fargs=(ax, trace, status_texts, drawbar_traces, patches_list),
                                      interval=self.dt*1000, blit=True, repeat=False)
        
        if os.environ.get('DISPLAY'):
            print("Showing simulation... Close the window to continue.")
            plt.show()
        else:
            print("Headless environment detected. Skipping plt.show().")
            
        if self.SAVE_ANIMATION:
            print("Saving animation...")
            writer = animation.PillowWriter(fps=20)
            ani.save('simulation_dynamic.gif', writer=writer)
            print("Simulation saved to simulation_dynamic.gif")
        else:
            print("Animation save skipped.")

if __name__ == "__main__":
    sim = TractorTrailerSimulator()
    sim.run_simulation()
    sim.animate()
