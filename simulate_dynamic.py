import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from dynamic_model import TractorTrailerDynamicModel

def simulate():
    # --- Parameters ---
    SAVE_ANIMATION = False
    PLOT_DRAWBAR_TRAJECTORY = True
    
    # Dimensions (meters)
    L0 = 1.28       # Tractor Wheelbase
    W = 1.09        # Track Width
    
    # Trailer Configuration (1 Drawbar Trailer consisting of Dolly and Trailer Body)
    trailers = [
        {'L_bar': 1.0, 'L_trl': 1.2, 'dh_prev': 0.62}, # Trailer 1 (dh_prev = d_h of tractor)
    ]
    
    num_trailers = len(trailers)
    
    # Vehicle Box Dimensions
    tractor_width = 1.3
    tractor_len = 2.950
    tractor_overhang = 0.62 
    
    trailer_width = 1.5
    trailer_body_len = 2.0
    trailer_overhang = 0.4 
    
    wheel_diam = 0.6
    wheel_width = 0.3

    dt = 0.05
    T = 20.0
    
    # Initialize Model
    model = TractorTrailerDynamicModel(L0, trailers, dt=dt, d_h=0.62)
    
    # Initial state: [x0, y0, theta0, theta1, theta2, vx, vy, r, rd, rt]
    # We start with the tractor moving at 2.0 m/s
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0])
    
    # Speed PI controller parameters
    v_target = 2.0
    Kp = 1500.0
    Ki = 500.0
    v_error_integral = 0.0
    
    # Simulation variables
    steps = int(T / dt)
    trajectory = []
    drawbar_trajectories = [] # List to store list of drawbar coords per step
    states = []
    inputs = []
    velocities = [] 
    
    for i in range(steps):
        t = i * dt
        
        # Speed Control (PI controller on Fxr)
        vx = state[5]
        v_error = v_target - vx
        v_error_integral += v_error * dt
        Fxr = Kp * v_error + Ki * v_error_integral
        # Clamp thrust force to realistic limits [-10000 N, 10000 N]
        Fxr = np.clip(Fxr, -10000.0, 10000.0)
        
        # Steering input: Smooth steady turn to drive in a circle (safe for heavy towing)
        delta = np.radians(8) * (1 - np.exp(-t))
        
        states.append(state)
        inputs.append([Fxr, delta])
        
        # Compute velocities using world-frame derivations from Section 1.3
        theta0, theta1, theta2 = state[2], state[3], state[4]
        vy, r, rd, rt = state[6], state[7], state[8], state[9]
        
        dx0 = vx * np.cos(theta0) - vy * np.sin(theta0)
        dy0 = vx * np.sin(theta0) + vy * np.cos(theta0)
        
        # Dolly velocity (Section 1.3)
        dxd = dx0 + model.d_h * r * np.sin(theta0) + model.l_fd * rd * np.sin(theta1)
        dyd = dy0 - model.d_h * r * np.cos(theta0) - model.l_fd * rd * np.cos(theta1)
        
        # Trailer body velocity (Section 1.3)
        dxt = dx0 + model.d_h * r * np.sin(theta0) + (model.l_fd + model.l_rd) * rd * np.sin(theta1) + model.l_ft * rt * np.sin(theta2)
        dyt = dy0 - model.d_h * r * np.cos(theta0) - (model.l_fd + model.l_rd) * rd * np.cos(theta1) - model.l_ft * rt * np.cos(theta2)
        
        # Project world velocity to trailer body longitudinal velocity
        vxt = dxt * np.cos(theta2) + dyt * np.sin(theta2)
        vyt = -dxt * np.sin(theta2) + dyt * np.cos(theta2)
        
        # Calculate Kinetic Energy (Section 1.4)
        T_tractor = 0.5 * model.m * (dx0**2 + dy0**2) + 0.5 * model.I_z * r**2
        T_dolly = 0.5 * model.m_d * (dxd**2 + dyd**2) + 0.5 * model.I_zd * rd**2
        T_trailer = 0.5 * model.m_t * (dxt**2 + dyt**2) + 0.5 * model.I_zt * rt**2
        T_total = T_tractor + T_dolly + T_trailer
        
        velocities.append([vxt, T_total])
        
        # Get Coordinates for Trajectory (Tractor Rear Axle)
        coords = model.get_coordinates(state)
        trajectory.append(coords[0]) # p0
        
        # Print state every 10 steps (0.5 seconds)
        if i % 10 == 0:
            x0_pos, y0_pos = coords[0][0], coords[0][1]
            xt_pos, yt_pos = coords[4][0], coords[4][1]
            v0_mag = np.sqrt(vx**2 + vy**2)
            vt_mag = np.sqrt(vxt**2 + vyt**2)
            print(f"t={t:05.2f}s | Tractor: x={x0_pos:06.2f}, y={y0_pos:06.2f}, v={v0_mag:05.2f} m/s | Trailer: x={xt_pos:06.2f}, y={yt_pos:06.2f}, v={vt_mag:05.2f} m/s")
        
        # Collect Drawbar Coordinates (Dolly Positions)
        current_drawbars = []
        for k in range(num_trailers):
            idx_dolly = 3 + 3*k
            current_drawbars.append(coords[idx_dolly])
        drawbar_trajectories.append(current_drawbars)
        
        state = model.update(state, Fxr, delta)
        
    trajectory = np.array(trajectory)
    drawbar_trajectories = np.array(drawbar_trajectories) # Shape: (steps, num_trailers, 2)
    states = np.array(states)
    inputs = np.array(inputs)
    velocities = np.array(velocities)
    
    # --- Visualization ---
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_aspect('equal')
    ax.set_xlim(-10, 30)
    ax.set_ylim(-10, 40)
    ax.grid(True)
    
    ax.set_title(f"Tractor-Trailer Dynamic Simulation ({num_trailers} Trailer)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    
    # Status Texts
    status_texts = []
    
    # 1. Tractor Text
    t_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=10, color='blue',
                     verticalalignment='top', fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6, edgecolor='none'))
    status_texts.append(t_text)
    
    # 2. Trailer Text
    cmap = plt.get_cmap('jet')
    trailer_colors = [cmap(float(k) / num_trailers) for k in range(num_trailers)]
    
    tr_text = ax.text(0.05, 0.90, '', transform=ax.transAxes, fontsize=10, color=trailer_colors[0],
                      verticalalignment='top', fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.6, edgecolor='none'))
    status_texts.append(tr_text)
    
    # Trace (Tractor)
    trace, = ax.plot([], [], 'b--', alpha=0.5, label='Tractor Path')
    
    # Drawbar Traces
    drawbar_traces = []
    if PLOT_DRAWBAR_TRAJECTORY:
        d_trace, = ax.plot([], [], '--', color=trailer_colors[0], alpha=0.4, linewidth=1)
        drawbar_traces.append(d_trace)

    # Drawing Helpers
    def draw_box(ax, center, length, width, angle, color='gray', alpha=0.5):
        c, s = np.cos(angle), np.sin(angle)
        dx, dy = -length/2, -width/2
        rx = dx*c - dy*s
        ry = dx*s + dy*c
        rect = Rectangle((center[0]+rx, center[1]+ry), length, width, angle=np.degrees(angle), color=color, alpha=alpha, ec='black')
        ax.add_patch(rect)
        return rect

    def draw_wheels_at_axle(ax, center, angle, track_width, steered_angle=0, color='black'):
        wl_pos = center + (track_width/2) * np.array([-np.sin(angle), np.cos(angle)])
        wr_pos = center - (track_width/2) * np.array([-np.sin(angle), np.cos(angle)])
        
        wa = angle + steered_angle
        
        w_objs = []
        for w_pos in [wl_pos, wr_pos]:
            w = draw_box(ax, w_pos, wheel_diam, wheel_width, wa, color=color, alpha=1.0)
            w_objs.append(w)
        
        l, = ax.plot([wl_pos[0], wr_pos[0]], [wl_pos[1], wr_pos[1]], 'k-', lw=2)
        return w_objs + [l]

    patches_list = []

    def update_plot(i):
        # Print status during animation (every 10 frames)
        if i % 10 == 0:
            vx, vy = states[i, 5], states[i, 6]
            v0_mag = np.sqrt(vx**2 + vy**2)
            
            theta0, theta1, theta2 = states[i, 2], states[i, 3], states[i, 4]
            r, rd, rt = states[i, 7], states[i, 8], states[i, 9]
            dx0 = vx * np.cos(theta0) - vy * np.sin(theta0)
            dy0 = vx * np.sin(theta0) + vy * np.cos(theta0)
            dxt = dx0 + model.d_h * r * np.sin(theta0) + (model.l_fd + model.l_rd) * rd * np.sin(theta1) + model.l_ft * rt * np.sin(theta2)
            dyt = dy0 - model.d_h * r * np.cos(theta0) - (model.l_fd + model.l_rd) * rd * np.cos(theta1) - model.l_ft * rt * np.cos(theta2)
            vt_mag = np.sqrt(dxt**2 + dyt**2)
            
            coords_frame = model.get_coordinates(states[i])
            x0_pos, y0_pos = coords_frame[0][0], coords_frame[0][1]
            xt_pos, yt_pos = coords_frame[4][0], coords_frame[4][1]
            t_current = i * dt
            print(f"[Animation] t={t_current:05.2f}s | Tractor: x={x0_pos:06.2f}, y={y0_pos:06.2f}, v={v0_mag:05.2f} m/s | Trailer: x={xt_pos:06.2f}, y={yt_pos:06.2f}, v={vt_mag:05.2f} m/s")

        for p in patches_list:
            p.remove()
        patches_list.clear()
        
        # Update Trace
        trace.set_data(trajectory[:i, 0], trajectory[:i, 1])
        
        # Update Drawbar Traces
        if PLOT_DRAWBAR_TRAJECTORY:
            drawbar_traces[0].set_data(drawbar_trajectories[:i, 0, 0], drawbar_trajectories[:i, 0, 1])
        
        state = states[i]
        Fxr_curr = inputs[i, 0]
        delta_curr = inputs[i, 1]
        vels = velocities[i]
        
        vx_curr = state[5]
        
        # Update Status Texts
        status_texts[0].set_text(f'Tractor Vx: {vx_curr:.2f} m/s\nForce Fxr: {Fxr_curr:.1f} N\nSteer: {np.degrees(delta_curr):.1f} deg')
        
        # Relative Drawbar Angle
        psi = state[2] - state[3]
        psi = (psi + np.pi) % (2 * np.pi) - np.pi
        
        T_total_display = vels[1]
        status_texts[1].set_text(f'Trailer Vxt: {vels[0]:.2f} m/s\nDrawbar Ang: {np.degrees(psi):.1f} deg\nKinetic Energy (T): {T_total_display/1000:.1f} kJ')
        
        # --- Coordinates ---
        coords = model.get_coordinates(state)
        
        p0 = coords[0]
        p0_f = coords[1]
        theta0 = state[2]
        
        # --- Tractor Body ---
        p_tractor_c = (p0 + p0_f) / 2
        patches_list.append(draw_box(ax, p_tractor_c, tractor_len, tractor_width, theta0, color='orangered', alpha=0.5))
        
        # Tractor Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p0, theta0, W)) # Rear
        patches_list.extend(draw_wheels_at_axle(ax, p0_f, theta0, W, steered_angle=delta_curr)) # Front
        
        # Tractor Tail Extension
        p_tr_rear_face = p0 - tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        h1 = coords[2]
        l_tr_tail, = ax.plot([p_tr_rear_face[0], h1[0]], [p_tr_rear_face[1], h1[1]], 'k-', lw=2)
        patches_list.append(l_tr_tail)
        
        # --- Trailer ---
        h_curr = coords[2]
        p_dolly = coords[3]
        p_axle = coords[4]
        
        theta_drawbar = state[3]
        theta_trailer = state[4]
        
        # Drawbar
        l_db, = ax.plot([h_curr[0], p_dolly[0]], [h_curr[1], p_dolly[1]], 'k-', lw=3)
        patches_list.append(l_db)
        
        # Trailer Body
        p_trailer_c = (p_dolly + p_axle) / 2
        patches_list.append(draw_box(ax, p_trailer_c, trailer_body_len, trailer_width, theta_trailer, color='blue', alpha=0.5))
        
        # Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p_dolly, theta_drawbar, W, color='black')) # Dolly
        patches_list.extend(draw_wheels_at_axle(ax, p_axle, theta_trailer, W, color='black')) # Rear
        
        # Hitch Point
        pt_h, = ax.plot(h_curr[0], h_curr[1], 'ko', ms=5)
        patches_list.append(pt_h)
        
        # Tail Extension
        p_tl_rear_face = p_axle - trailer_overhang * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
        p_stub = p_tl_rear_face - 0.1 * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
        l_tail, = ax.plot([p_tl_rear_face[0], p_stub[0]], [p_tl_rear_face[1], p_stub[1]], 'k-', lw=2)
        patches_list.append(l_tail)

        return patches_list + [trace] + status_texts + drawbar_traces
 
    ani = animation.FuncAnimation(fig, update_plot, frames=len(states), interval=dt*1000, blit=True, repeat=False)
    
    if os.environ.get('DISPLAY'):
        print("Showing simulation... Close the window to continue.")
        plt.show()
    else:
        print("Headless environment detected. Skipping plt.show().")
        
    if SAVE_ANIMATION:
        print("Saving animation...")
        writer = animation.PillowWriter(fps=20)
        ani.save('simulation_dynamic.gif', writer=writer)
        print("Simulation saved to simulation_dynamic.gif")
    else:
        print("Animation save skipped.")

if __name__ == "__main__":
    simulate()
