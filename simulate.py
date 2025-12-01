import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from kinematic_model import TractorTrailerModel

def simulate():
    # --- Parameters ---
    SAVE_ANIMATION = True
    
    # Dimensions (meters)
    L0 = 2.0        # Tractor Wheelbase
    W = 1.0         # Track Width
    
    # Vehicle Box Dimensions (User Request)
    tractor_width = 1.5
    tractor_len = 2.8
    trailer_width = 1.5
    trailer_len = 2.0
    
    # Tractor Geometry
    # Center body on wheelbase for symmetry (like trailer) or keep previous?
    # Previous: Rear Overhang 0.4. Front Overhang 0.4.
    # This is symmetric.
    tractor_overhang = (tractor_len - L0) / 2 # 0.4
    tail_ext = 0.15
    dh = tractor_overhang + tail_ext # Hitch position
    
    # Trailer Geometry
    L1 = 1.0        # Drawbar Length
    L2 = 1.2        # Dolly Axle to Trailer Axle
    trailer_body_len = trailer_len
    # Equal margins:
    trailer_overhang = (trailer_body_len - L2) / 2 # 0.25
    
    # Wheel Radius
    r_wheel = 10 * 0.0254
    wheel_diam = 2 * r_wheel
    wheel_width = 0.3

    dt = 0.05
    T = 30.0
    
    model = TractorTrailerModel(L0, dh, L1, L2, dt)
    
    # Initial state
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Simulation
    steps = int(T / dt)
    trajectory = []
    states = []
    inputs = []
    
    for i in range(steps):
        t = i * dt
        v0 = 2.0
        # Infinity-like pattern
        delta = np.radians(30) * np.sin(0.5 * t)
        
        states.append(state)
        inputs.append([v0, delta])
        p0, p0_f, h, p1, p2 = model.get_coordinates(state)
        trajectory.append(p0)
        state = model.update(state, v0, delta)
        
    trajectory = np.array(trajectory)
    states = np.array(states)
    inputs = np.array(inputs)
    
    # --- Visualization ---
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_aspect('equal')
    ax.set_xlim(-5, 25)
    ax.set_ylim(-5, 40)
    ax.grid(True)
    
    ax.set_title("Tractor-Trailer Simulation")
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
        
        # Update Status Text
        status_text.set_text(f'Velocity: {v_curr:.2f} m/s\nSteering: {np.degrees(delta_curr):.2f} deg')
        
        x0, y0, theta0, theta1, theta2 = state
        
        # --- Coordinates ---
        p0 = np.array([x0, y0]) # Tractor Rear Axle
        p0_f = p0 + L0 * np.array([np.cos(theta0), np.sin(theta0)]) # Tractor Front Axle
        h = p0 - dh * np.array([np.cos(theta0), np.sin(theta0)]) # Hitch
        p1 = h - L1 * np.array([np.cos(theta1), np.sin(theta1)]) # Dolly Axle
        p2 = p1 - L2 * np.array([np.cos(theta2), np.sin(theta2)]) # Trailer Axle
        
        # --- Tractor Body ---
        # Centered on Wheelbase (Midpoint of p0 and p0_f)
        p_tractor_c = (p0 + p0_f) / 2
        patches_list.append(draw_box(ax, p_tractor_c, tractor_len, tractor_width, theta0, color='lightblue'))
        
        # Tractor Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p0, theta0, W)) # Rear
        # Use stored delta for visualization
        patches_list.extend(draw_wheels_at_axle(ax, p0_f, theta0, W, steered_angle=delta_curr)) # Front
        
        # Tractor Tail Extension
        # Rear Face is p0 - overhang * forward
        p_tr_rear_face = p0 - tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        l_tr_tail, = ax.plot([p_tr_rear_face[0], h[0]], [p_tr_rear_face[1], h[1]], 'k-', lw=2)
        patches_list.append(l_tr_tail)
        
        # --- Drawbar ---
        l_db, = ax.plot([h[0], p1[0]], [h[1], p1[1]], 'k-', lw=3, color='gray')
        patches_list.append(l_db)
        
        # --- Trailer Body ---
        # Centered on Wheelbase (Midpoint of p1 and p2)
        p_trailer_c = (p1 + p2) / 2
        patches_list.append(draw_box(ax, p_trailer_c, trailer_body_len, trailer_width, theta2, color='lightcoral'))
        
        # Trailer Wheels
        patches_list.extend(draw_wheels_at_axle(ax, p1, theta1, W, color='black')) # Dolly
        patches_list.extend(draw_wheels_at_axle(ax, p2, theta2, W, color='black')) # Rear
        
        # Trailer Tail Extension
        # Rear Face is p2 - overhang * forward
        p_tl_rear_face = p2 - trailer_overhang * np.array([np.cos(theta2), np.sin(theta2)])
        # Tail Hitch is Rear Face - tail_ext * forward
        p_h2 = p_tl_rear_face - tail_ext * np.array([np.cos(theta2), np.sin(theta2)])
        
        l_tl_tail, = ax.plot([p_tl_rear_face[0], p_h2[0]], [p_tl_rear_face[1], p_h2[1]], 'k-', lw=2)
        patches_list.append(l_tl_tail)
        
        # Hitch Points
        h1, = ax.plot(h[0], h[1], 'ko', ms=5)
        h2, = ax.plot(p_h2[0], p_h2[1], 'ko', ms=5)
        patches_list.extend([h1, h2])
        
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
