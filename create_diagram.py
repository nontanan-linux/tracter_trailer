import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse

def create_diagram(L0=2.0, L1=1.0, L2=1.2, W=1.5, tractor_width=1.5, tractor_len=2.8, trailer_width=1.5, trailer_len=2.0, tail_ext=0.15, save_path='kinematic_diagram_full.png'):
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    ax.axis('off')

    # --- Calculations ---
    # Tractor Geometry
    tractor_len = 2.8
    tractor_overhang = (tractor_len - L0) / 2
    dh = tractor_overhang + tail_ext 
    
    # Trailer Geometry
    trailer_overhang = (trailer_len - L2) / 2
    
    # Wheel Dimensions
    r_wheel = 10 * 0.0254
    wheel_diam = 2 * r_wheel
    wheel_width = 0.3

    # Visualization Angles
    theta0 = np.radians(30)
    theta1 = np.radians(10)
    theta2 = np.radians(-5)
    # delta is not shown
    delta = np.radians(25) # Still needed for wheel drawing
    
    # Starting Position
    x0, y0 = 0, 0  # Tractor Rear Axle Center (Baselink 0)

    # --- Kinematic Points ---
    
    # Tractor
    # Rear Axle: (x0, y0)
    # Front Axle:
    x0_f = x0 + L0 * np.cos(theta0)
    y0_f = y0 + L0 * np.sin(theta0)
    # Hitch:
    xh = x0 - dh * np.cos(theta0)
    yh = y0 - dh * np.sin(theta0)
    
    # Dolly (Front Axle of Trailer)
    x1 = xh - L1 * np.cos(theta1)
    y1 = yh - L1 * np.sin(theta1)
    
    # Trailer Rear Axle
    x2 = x1 - L2 * np.cos(theta2)
    y2 = y1 - L2 * np.sin(theta2)
    
    # Trailer Tail Hitch
    dist_tail_hitch = trailer_overhang + tail_ext
    x_tail = x2 - dist_tail_hitch * np.cos(theta2)
    y_tail = y2 - dist_tail_hitch * np.sin(theta2)

    # --- Drawing Helpers ---
    
    def draw_wheel_pair(ax, center, angle, width, steered_angle=0, color='black'):
        cx, cy = center
        px = -np.sin(angle)
        py = np.cos(angle)
        
        wl_x = cx + (width/2) * px
        wl_y = cy + (width/2) * py
        wr_x = cx - (width/2) * px
        wr_y = cy - (width/2) * py
        
        ax.plot([wl_x, wr_x], [wl_y, wr_y], 'k-', lw=2)
        
        wa = angle + steered_angle
        
        for wx, wy in [(wl_x, wl_y), (wr_x, wr_y)]:
            c, s = np.cos(wa), np.sin(wa)
            dx, dy = -wheel_diam/2, -wheel_width/2
            rx = dx*c - dy*s
            ry = dx*s + dy*c
            rect = patches.Rectangle((wx+rx, wy+ry), wheel_diam, wheel_width, angle=np.degrees(wa), color=color)
            ax.add_patch(rect)

    def draw_chassis(ax, center, length, width, angle, color='gray', label=None):
        cx, cy = center
        c, s = np.cos(angle), np.sin(angle)
        dx, dy = -length/2, -width/2
        rx = dx*c - dy*s
        ry = dx*s + dy*c
        rect = patches.Rectangle((cx+rx, cy+ry), length, width, angle=np.degrees(angle), 
                                 linewidth=2, edgecolor='black', facecolor=color, alpha=0.3, label=label)
        ax.add_patch(rect)

    def draw_dashed_ref(ax, center, length=3):
        cx, cy = center
        ax.plot([cx, cx + length], [cy, cy], 'k--', lw=1, alpha=0.6)

    # --- Draw Vehicles ---

    # 1. Tractor
    p_tr_front_face = np.array([x0_f, y0_f]) + tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
    p_tr_rear_face = np.array([x0, y0]) - tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
    p_tr_c = (p_tr_front_face + p_tr_rear_face) / 2
    draw_chassis(ax, p_tr_c, tractor_len, tractor_width, theta0, color='lightblue', label='Tractor')
    
    draw_wheel_pair(ax, (x0_f, y0_f), theta0, W, steered_angle=delta, color='black')
    draw_wheel_pair(ax, (x0, y0), theta0, W, color='black')
    
    ax.plot([p_tr_rear_face[0], xh], [p_tr_rear_face[1], yh], 'k-', lw=2)
    
    # 2. Drawbar
    ax.plot([xh, x1], [yh, y1], 'k-', lw=4, color='gray')
    
    # 3. Trailer
    p_tl_c = (np.array([x1, y1]) + np.array([x2, y2])) / 2
    draw_chassis(ax, p_tl_c, trailer_len, trailer_width, theta2, color='lightcoral', label='Trailer')
    
    draw_wheel_pair(ax, (x1, y1), theta1, W, color='black')
    draw_wheel_pair(ax, (x2, y2), theta2, W, color='black')
    
    p_tl_rear_face = np.array([x2, y2]) - trailer_overhang * np.array([np.cos(theta2), np.sin(theta2)])
    ax.plot([p_tl_rear_face[0], x_tail], [p_tl_rear_face[1], y_tail], 'k-', lw=2)
    
    ax.plot(xh, yh, 'ko', ms=6)
    ax.plot(x_tail, y_tail, 'ko', ms=6)

    # --- Annotations & Reference Lines ---
    
    # Centerline
    ax.plot([x0, x0_f], [y0, y0_f], 'k-.', lw=1, alpha=0.4)
    ax.plot([x1, x2], [y1, y2], 'k-.', lw=1, alpha=0.4)

    # Reference Lines (Horizontal)
    draw_dashed_ref(ax, (x0, y0)) # At Tractor Rear Axle
    draw_dashed_ref(ax, (xh, yh)) # At Hitch (for Drawbar angle)
    draw_dashed_ref(ax, (x2, y2)) # At Trailer Rear Axle

    # Points
    ax.text(x0, y0-1, '$(x_0, y_0)$', color='blue')
    ax.text(xh, yh+0.5, 'Hitch', fontsize=8)
    ax.text(x2, y2-1, '$(x_2, y_2)$', color='red')

    # Dimensions
    ax.annotate(f'$L_0={L0}$', xy=((x0+x0_f)/2, (y0+y0_f)/2), xytext=(-10, 10), textcoords='offset points')
    ax.annotate(f'$L_1={L1}$', xy=((xh+x1)/2, (yh+y1)/2), xytext=(-10, 10), textcoords='offset points')
    ax.annotate(f'$L_2={L2}$', xy=((x1+x2)/2, (y1+y2)/2), xytext=(-10, 10), textcoords='offset points')
    
    # Angles
    # Theta0 at Tractor Rear Axle (Baselink)
    patches.Arc((x0, y0), 3, 3, angle=0, theta1=0, theta2=np.degrees(theta0), color='blue')
    ax.text(x0+2, y0+0.5, r'$\theta_0$', color='blue')
    
    # Theta1 at Hitch (Baselink of Drawbar)
    patches.Arc((xh, yh), 3, 3, angle=0, theta1=0, theta2=np.degrees(theta1), color='green')
    ax.text(xh+2, yh+0.5, r'$\theta_1$', color='green')
    
    # Theta2 at Trailer Rear Axle (Baselink of Trailer)
    patches.Arc((x2, y2), 3, 3, angle=0, theta1=0, theta2=np.degrees(theta2), color='red')
    ax.text(x2+2, y2-0.5, r'$\theta_2$', color='red')
    
    # Delta removed as requested
    plt.xlim(-10, 50)
    plt.ylim(-10, 50)
    plt.title("Kinematic Diagram: Tractor + Drawbar Trailer")
    plt.tight_layout()
    plt.savefig(save_path)
    print(f"Diagram saved to {save_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate Kinematic Diagram')
    parser.add_argument('--L0', type=float, default=2.0, help='Tractor Wheelbase')
    parser.add_argument('--L1', type=float, default=1.0, help='Drawbar Length')
    parser.add_argument('--L2', type=float, default=1.2, help='Trailer Wheelbase')
    parser.add_argument('--W', type=float, default=1.0, help='Track Width')
    parser.add_argument('--tractor_width', type=float, default=1.5, help='Tractor Body Width')
    parser.add_argument('--tractor_len', type=float, default=2.8, help='Tractor Body Length')
    parser.add_argument('--trailer_width', type=float, default=1.5, help='Trailer Body Width')
    parser.add_argument('--trailer_len', type=float, default=2.0, help='Trailer Body Length')
    parser.add_argument('--tail_ext', type=float, default=0.15, help='Tail Extension')
    
    args = parser.parse_args()
    
    create_diagram(L0=args.L0, L1=args.L1, L2=args.L2, W=args.W, 
                   tractor_width=args.tractor_width, tractor_len=args.tractor_len,
                   trailer_width=args.trailer_width, trailer_len=args.trailer_len, 
                   tail_ext=args.tail_ext)
