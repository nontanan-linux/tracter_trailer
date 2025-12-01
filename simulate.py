import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from kinematic_model import TractorTrailerModel

def simulate():
    # --- Parameters ---
    SAVE_ANIMATION = False
    
    # Dimensions (meters)
    L0 = 2.0        # Tractor Wheelbase
    W = 1.0         # Track Width
    
    dh = 0.5        # Hitch 1 offset (behind rear axle)
    L1 = 1.0        # Drawbar 1 Length
    L2 = 1.2        # Trailer 1 Length (Dolly to Axle)
    
    dh2 = 0.5       # Hitch 2 offset (behind Trailer 1 axle)
    L3 = 1.0        # Drawbar 2 Length
    L4 = 1.2        # Trailer 2 Length (Dolly to Axle)
    
    # Vehicle Box Dimensions (User Request)
    tractor_width = 1.5
    tractor_len = 2.8
    tractor_overhang = 0.4 # Distance from rear axle to rear face
    
    trailer_width = 1.5
    trailer_len = 2.0
    trailer_body_len = trailer_len
    trailer_overhang = 0.4 # Distance from rear axle to rear face
    tail_ext = 0.1 # Small extension for hitch point
    
    wheel_diam = 0.6
    wheel_width = 0.3

    dt = 0.05
    T = 30.0
    
    model = TractorTrailerModel(L0, dh, L1, L2, dh2, L3, L4, dt)
    
    # Initial state [x0, y0, theta0, theta1, theta2, theta3, theta4]
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Simulation
    steps = int(T / dt)
    trajectory = []
    states = []
    inputs = []
    velocities = [] # [v1, v2, v3, v4]
    
    for i in range(steps):
        t = i * dt
        v0 = 2.0
        # Infinity-like pattern
        delta = np.radians(30) * np.sin(0.5 * t)
        
        states.append(state)
        inputs.append([v0, delta])
        
        # Get full kinematic vector
        q_kin = model.get_kinematic_vector(state, v0, delta)
        # Extract velocities: v1 (3), v2 (5), v3 (7), v4 (9)
        velocities.append([q_kin[3], q_kin[5], q_kin[7], q_kin[9]])
        
        # Unpack all coordinates
        p0, p0_f, h1, p1, p2, h2, p3, p4 = model.get_coordinates(state)
        trajectory.append(p0)
        state = model.update(state, v0, delta)
        
    trajectory = np.array(trajectory)
    states = np.array(states)
    inputs = np.array(inputs)
    velocities = np.array(velocities)
    
    # --- Visualization ---
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_aspect('equal')
    ax.set_xlim(-10, 30)
    ax.set_ylim(-10, 40)
    ax.grid(True)
    
    ax.set_title("Tractor-Trailer Simulation (Double Trailer)")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    
    # Status Text
    status_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12,
                          verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Trace
    trace, = ax.plot([], [], 'b--', alpha=0.5)

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
        for p in patches_list:
            p.remove()
        patches_list.clear()
        
        # Update Trace
        trace.set_data(trajectory[:i, 0], trajectory[:i, 1])
        
        state = states[i]
        v_curr = inputs[i, 0]
        delta_curr = inputs[i, 1]
        vels = velocities[i] # [v1, v2, v3, v4]
        
        # Update Status Text
        status_text.set_text(f'Tractor V: {v_curr:.2f} m/s\nSteering: {np.degrees(delta_curr):.2f} deg\n'
                             f'Dolly 1 V: {vels[0]:.2f} m/s\nTrailer 1 V: {vels[1]:.2f} m/s\n'
                             f'Dolly 2 V: {vels[2]:.2f} m/s\nTrailer 2 V: {vels[3]:.2f} m/s')
        
        x0, y0, theta0, theta1, theta2, theta3, theta4 = state
        
        # --- Coordinates ---
        p0, p0_f, h1, p1, p2, h2, p3, p4 = model.get_coordinates(state)
        
        # --- Tractor Body ---
        # Centered on Wheelbase (Midpoint of p0 and p0_f)
        p_tractor_c = (p0 + p0_f) / 2
        patches_list.append(draw_box(ax, p_tractor_c, tractor_len, tractor_width, theta0, color='red', alpha=0.5))
        
        # Tractor Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p0, theta0, W)) # Rear
        patches_list.extend(draw_wheels_at_axle(ax, p0_f, theta0, W, steered_angle=delta_curr)) # Front
        
        # Tractor Tail Extension
        # Rear Face is p0 - overhang * forward
        p_tr_rear_face = p0 - tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        l_tr_tail, = ax.plot([p_tr_rear_face[0], h1[0]], [p_tr_rear_face[1], h1[1]], 'k-', lw=2)
        patches_list.append(l_tr_tail)
        
        # --- Drawbar 1 ---
        l_db1, = ax.plot([h1[0], p1[0]], [h1[1], p1[1]], 'k-', lw=3, color='gray')
        patches_list.append(l_db1)
        
        # --- Trailer 1 Body ---
        # Centered on Wheelbase (Midpoint of p1 and p2)
        p_trailer1_c = (p1 + p2) / 2
        patches_list.append(draw_box(ax, p_trailer1_c, trailer_body_len, trailer_width, theta2, color='blue', alpha=0.5))
        
        # Trailer 1 Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p1, theta1, W, color='black')) # Dolly
        patches_list.extend(draw_wheels_at_axle(ax, p2, theta2, W, color='black')) # Rear
        
        # Trailer 1 Tail Extension (to Hitch 2)
        # Rear Face is p2 - overhang * forward
        p_tl1_rear_face = p2 - trailer_overhang * np.array([np.cos(theta2), np.sin(theta2)])
        l_tl1_tail, = ax.plot([p_tl1_rear_face[0], h2[0]], [p_tl1_rear_face[1], h2[1]], 'k-', lw=2)
        patches_list.append(l_tl1_tail)
        
        # --- Drawbar 2 ---
        l_db2, = ax.plot([h2[0], p3[0]], [h2[1], p3[1]], 'k-', lw=3, color='gray')
        patches_list.append(l_db2)
        
        # --- Trailer 2 Body ---
        # Centered on Wheelbase (Midpoint of p3 and p4)
        p_trailer2_c = (p3 + p4) / 2
        patches_list.append(draw_box(ax, p_trailer2_c, trailer_body_len, trailer_width, theta4, color='blue', alpha=0.5))
        
        # Trailer 2 Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p3, theta3, W, color='black')) # Dolly
        patches_list.extend(draw_wheels_at_axle(ax, p4, theta4, W, color='black')) # Rear
        
        # Trailer 2 Tail Extension
        p_tl2_rear_face = p4 - trailer_overhang * np.array([np.cos(theta4), np.sin(theta4)])
        # Just a small stub for visual consistency
        p_h3_stub = p_tl2_rear_face - tail_ext * np.array([np.cos(theta4), np.sin(theta4)])
        l_tl2_tail, = ax.plot([p_tl2_rear_face[0], p_h3_stub[0]], [p_tl2_rear_face[1], p_h3_stub[1]], 'k-', lw=2)
        patches_list.append(l_tl2_tail)
        
        # Hitch Points
        pt_h1, = ax.plot(h1[0], h1[1], 'ko', ms=5)
        pt_h2, = ax.plot(h2[0], h2[1], 'ko', ms=5)
        patches_list.extend([pt_h1, pt_h2])
        
        return patches_list + [trace, status_text]

    ani = animation.FuncAnimation(fig, update_plot, frames=len(states), interval=dt*1000, blit=True, repeat=False)
    
    print("Showing simulation... Close the window to continue.")
    plt.show()
    
    if SAVE_ANIMATION:
        print("Saving animation...")
        writer = animation.PillowWriter(fps=20)
        ani.save('simulation_full.gif', writer=writer)
        print("Simulation saved to simulation_full.gif")
    else:
        print("Animation save skipped.")

if __name__ == "__main__":
    simulate()
