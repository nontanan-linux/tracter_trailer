import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from dynamic_model import TractorTrailerDynamicModel
import sys
import os

class TractorTrailerSimulator:
    def __init__(self):
        self.dt = 0.05
        self.t_end = 20.0
        
        self.model = TractorTrailerDynamicModel()
        
        # Dimensions for drawing
        self.tractor_len = 4.0
        self.tractor_width = 2.0
        self.tractor_overhang = 1.0 # Distance from rear axle to rear edge
        
        self.trailer_body_len = 4.0
        self.trailer_width = 2.0
        self.trailer_overhang = 1.0 # Distance from rear axle to rear edge of trailer
        
        self.wheel_diam = 0.8
        self.wheel_width = 0.3
        self.W = 1.8 # Track width
        
        self.SAVE_ANIMATION = True
        
        self.states = []
        self.inputs = []
        self.hitch_forces = []
        
        self.trajectory = []
        self.drawbar_trajectories = []

    def get_coordinates(self, state):
        pos = state['positions']
        
        x0, y0, theta0 = pos[0], pos[1], pos[2]
        xd, yd, theta1 = pos[3], pos[4], pos[5]
        xt, yt, theta2 = pos[6], pos[7], pos[8]
        
        p0 = np.array([x0, y0])
        p0_f = p0 + self.model.l_f * np.array([np.cos(theta0), np.sin(theta0)])
        
        h1 = p0 - self.model.d_h * np.array([np.cos(theta0), np.sin(theta0)])
        p_axle_f = np.array([xd, yd])  # Drawbar axle = Trailer Front Axle
        p_axle_r = p_axle_f - self.model.l_ft * np.array([np.cos(theta2), np.sin(theta2)]) - self.model.l_rt * np.array([np.cos(theta2), np.sin(theta2)])
        
        # Actually, Trailer CG is at (xt, yt), so rear axle is at (xt - l_rt*cos(theta2), yt - l_rt*sin(theta2))
        p_axle_r = np.array([xt - self.model.l_rt * np.cos(theta2), yt - self.model.l_rt * np.sin(theta2)])
        
        return [p0, p0_f, h1, p_axle_f, p_axle_r]

    def run_simulation(self):
        # Initial State [x0, y0, theta0, xd, yd, theta1, xt, yt, theta2]
        initial_pos = np.array([0.0, 0.0, 0.0, 
                                -self.model.d_h - self.model.L_bar, 0.0, 0.0,
                                -self.model.d_h - self.model.L_bar - self.model.l_ft, 0.0, 0.0])
                                
        # Initial Velocities [v_x, v_y, r, v_xd, v_yd, r_d, v_xt, v_yt, r_t]
        v0 = 5.0
        initial_vel = np.array([v0, 0.0, 0.0, v0, 0.0, 0.0, v0, 0.0, 0.0])
        
        state = {'positions': initial_pos, 'velocities': initial_vel}
        
        num_steps = int(self.t_end / self.dt)
        print(f"Running simulation for {num_steps} steps ({self.t_end}s)...")
        
        for i in range(num_steps):
            t = i * self.dt
            
            # Simple lane change / steering profile
            if 2.0 <= t < 6.0:
                delta = np.radians(10.0)
            elif 6.0 <= t < 10.0:
                delta = np.radians(-10.0)
            else:
                delta = 0.0
                
            self.states.append({'positions': state['positions'].copy(), 'velocities': state['velocities'].copy()})
            self.inputs.append([0.0, delta]) # F_xf = 0, delta
            
            coords = self.get_coordinates(state)
            self.trajectory.append(coords[0])
            self.drawbar_trajectories.append(coords[3])
            
            res = self.model.step(state, delta, dt=self.dt)
            state = {'positions': res['positions'], 'velocities': res['velocities']}
            self.hitch_forces.append(res['hitch_forces'])
            
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

    def update_plot(self, i, ax, trace, d_trace, status_texts, patches_list):
        for p in patches_list:
            p.remove()
        patches_list.clear()
        
        tr_x = [p[0] for p in self.trajectory[:i]]
        tr_y = [p[1] for p in self.trajectory[:i]]
        trace.set_data(tr_x, tr_y)
        
        db_x = [p[0] for p in self.drawbar_trajectories[:i]]
        db_y = [p[1] for p in self.drawbar_trajectories[:i]]
        d_trace.set_data(db_x, db_y)
            
        state = self.states[i]
        delta_curr = self.inputs[i][1]
        
        pos = state['positions']
        vels = state['velocities']
        
        vx_curr = vels[0]
        vxt_curr = vels[6]
        
        if i % 10 == 0:
            print(f"t={i*self.dt:05.2f}s | Tractor: v={vx_curr:05.2f} m/s | Trailer: v={vxt_curr:05.2f} m/s")
            sys.stdout.flush()
            
        status_texts[0].set_text(f'Tractor Vx: {vx_curr:.2f} m/s\nSteer: {np.degrees(delta_curr):.1f} deg')
        
        psi = pos[2] - pos[5] # theta0 - theta1 (Drawbar relative to Tractor)
        psi = (psi + np.pi) % (2 * np.pi) - np.pi
        
        status_texts[1].set_text(f'Trailer Vxt: {vxt_curr:.2f} m/s\nDrawbar Ang: {np.degrees(psi):.1f} deg')
        
        coords = self.get_coordinates(state)
        p0 = coords[0]
        p0_f = coords[1]
        h1 = coords[2]
        p_axle_f = coords[3] # Front steerable axle of trailer (Drawbar Axle)
        p_axle_r = coords[4] # Rear fixed axle of trailer
        
        theta0 = pos[2]
        theta1 = pos[5]
        theta2 = pos[8]
        
        # Draw Tractor
        p_tractor_c = p0 + ((self.tractor_len / 2) - self.tractor_overhang) * np.array([np.cos(theta0), np.sin(theta0)])
        patches_list.append(self.draw_box(ax, p_tractor_c, self.tractor_len, self.tractor_width, theta0, color='orangered', alpha=0.5))
        
        patches_list.extend(self.draw_wheels_at_axle(ax, p0, theta0, self.W)) 
        patches_list.extend(self.draw_wheels_at_axle(ax, p0_f, theta0, self.W, steered_angle=delta_curr)) 
        
        # Tractor rear overhang to Hitch
        p_tr_rear_face = p0 - self.tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        l_tr_tail, = ax.plot([p_tr_rear_face[0], h1[0]], [p_tr_rear_face[1], h1[1]], 'k-', lw=2)
        patches_list.append(l_tr_tail)
        
        # Draw Drawbar (from Hitch to Front Axle of Trailer)
        l_db, = ax.plot([h1[0], p_axle_f[0]], [h1[1], p_axle_f[1]], 'k-', lw=3)
        patches_list.append(l_db)
        
        # Draw Trailer Body (centered between front and rear axles)
        p_trailer_c = p_axle_r + (self.trailer_body_len / 2 - self.trailer_overhang) * np.array([np.cos(theta2), np.sin(theta2)])
        patches_list.append(self.draw_box(ax, p_trailer_c, self.trailer_body_len, self.trailer_width, theta2, color='blue', alpha=0.5))
        
        # Draw Trailer Front Axle (Steers according to drawbar angle theta1)
        patches_list.extend(self.draw_wheels_at_axle(ax, p_axle_f, theta1, self.W)) 
        
        # Draw Trailer Rear Axle (Fixed to trailer body angle theta2)
        patches_list.extend(self.draw_wheels_at_axle(ax, p_axle_r, theta2, self.W))  
        
        # Draw Hitch
        pt_h, = ax.plot(h1[0], h1[1], 'ko', ms=5)
        patches_list.append(pt_h)
        
        return patches_list + [trace, d_trace] + status_texts

    def animate(self):
        fig, ax = plt.subplots(figsize=(12, 12))
        ax.set_aspect('equal')
        ax.set_xlim(-10, 40)
        ax.set_ylim(-10, 50)
        ax.grid(True)
        
        ax.set_title(f"Tractor-Trailer Dynamic Simulation (9-DOF Lagrangian Matrix)")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        
        status_texts = []
        t_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=10, color='blue',
                         verticalalignment='top', fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6, edgecolor='none'))
        status_texts.append(t_text)
        
        tr_text = ax.text(0.05, 0.82, '', transform=ax.transAxes, fontsize=10, color='orange',
                          verticalalignment='top', fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6, edgecolor='none'))
        status_texts.append(tr_text)
        
        trace, = ax.plot([], [], 'b--', alpha=0.5, label='Tractor Path')
        d_trace, = ax.plot([], [], '--', color='orange', alpha=0.4, linewidth=1)
            
        patches_list = []
        
        ani = animation.FuncAnimation(fig, self.update_plot, frames=len(self.states), 
                                      fargs=(ax, trace, d_trace, status_texts, patches_list),
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

if __name__ == "__main__":
    sim = TractorTrailerSimulator()
    sim.run_simulation()
    sim.animate()
