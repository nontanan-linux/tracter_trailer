import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse

def create_diagram(L0=2.0, L1=1.0, L2=1.2, L3=1.0, L4=1.2, W=1.5, 
                   tractor_width=1.5, tractor_len=2.8, 
                   trailer_width=1.5, trailer_len=2.0, 
                   tail_ext=0.15, save_path='kinematic_diagram_full.png'):
    fig, ax = plt.subplots(figsize=(16, 10))
    ax.set_aspect('equal')
    ax.axis('off')

    # --- Calculations ---
    # Tractor Geometry
    tractor_overhang = (tractor_len - L0) / 2
    dh = tractor_overhang + tail_ext 
    
    # Trailer 1 Geometry
    trailer_overhang = (trailer_len - L2) / 2
    dh2 = trailer_overhang + tail_ext # Hitch 2 offset
    
    # Trailer 2 Geometry
    trailer2_len = trailer_len
    trailer2_width = trailer_width
    trailer2_overhang = (trailer2_len - L4) / 2

    # Wheel Dimensions
    r_wheel = 10 * 0.0254
    wheel_diam = 2 * r_wheel
    wheel_width = 0.3

    # Visualization Angles
    theta0 = np.radians(30)
    theta1 = np.radians(10)
    theta2 = np.radians(-5)
    theta3 = np.radians(5)
    theta4 = np.radians(-10)
    
    delta = np.radians(25) 
    
    # Starting Position
    x0, y0 = 0, 0  # Tractor Rear Axle Center (Baselink 0)

    # --- Kinematic Points ---
    
    # Tractor
    # Front Axle:
    x0_f = x0 + L0 * np.cos(theta0)
    y0_f = y0 + L0 * np.sin(theta0)
    # Hitch 1:
    xh1 = x0 - dh * np.cos(theta0)
    yh1 = y0 - dh * np.sin(theta0)
    
    # Trailer 1
    # Dolly 1 (Front Axle of Trailer 1)
    x1 = xh1 - L1 * np.cos(theta1)
    y1 = yh1 - L1 * np.sin(theta1)
    # Trailer 1 Rear Axle
    x2 = x1 - L2 * np.cos(theta2)
    y2 = y1 - L2 * np.sin(theta2)
    # Hitch 2
    xh2 = x2 - dh2 * np.cos(theta2)
    yh2 = y2 - dh2 * np.sin(theta2)
    
    # Trailer 2
    # Dolly 2 (Front Axle of Trailer 2)
    x3 = xh2 - L3 * np.cos(theta3)
    y3 = yh2 - L3 * np.sin(theta3)
    # Trailer 2 Rear Axle
    x4 = x3 - L4 * np.cos(theta4)
    y4 = y3 - L4 * np.sin(theta4)
    
    # Trailer 2 Tail
    dist_tail_hitch2 = trailer2_overhang + tail_ext
    x_tail2 = x4 - dist_tail_hitch2 * np.cos(theta4)
    y_tail2 = y4 - dist_tail_hitch2 * np.sin(theta4)

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

    def draw_dashed_ref(ax, center, length=1.0, angle=0):
        cx, cy = center
        dx = length * np.cos(angle)
        dy = length * np.sin(angle)
        ax.plot([cx, cx + dx], [cy, cy + dy], 'k--', lw=1, alpha=0.6)

    # --- Draw Vehicles ---

    # 1. Tractor
    p_tr_front_face = np.array([x0_f, y0_f]) + tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
    p_tr_rear_face = np.array([x0, y0]) - tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
    p_tr_c = (p_tr_front_face + p_tr_rear_face) / 2
    draw_chassis(ax, p_tr_c, tractor_len, tractor_width, theta0, color='orangered', label='Tractor')
    
    draw_wheel_pair(ax, (x0_f, y0_f), theta0, W, steered_angle=delta, color='black')
    draw_wheel_pair(ax, (x0, y0), theta0, W, color='black')
    
    ax.plot([p_tr_rear_face[0], xh1], [p_tr_rear_face[1], yh1], 'k-', lw=2)
    
    # 2. Drawbar 1
    ax.plot([xh1, x1], [yh1, y1], 'k-', lw=4)
    
    # 3. Trailer 1
    p_tl1_rear_face = np.array([x2, y2]) - trailer_overhang * np.array([np.cos(theta2), np.sin(theta2)])
    p_tl1_front_face = np.array([x1, y1]) + trailer_overhang * np.array([np.cos(theta2), np.sin(theta2)]) # Approx
    # Better center calculation:
    p_tl1_c = (np.array([x1, y1]) + np.array([x2, y2])) / 2
    draw_chassis(ax, p_tl1_c, trailer_len, trailer_width, theta2, color='blue', label='Trailer 1')
    
    draw_wheel_pair(ax, (x1, y1), theta1, W, color='black')
    draw_wheel_pair(ax, (x2, y2), theta2, W, color='black')
    
    ax.plot([p_tl1_rear_face[0], xh2], [p_tl1_rear_face[1], yh2], 'k-', lw=2)
    
    # 4. Drawbar 2
    ax.plot([xh2, x3], [yh2, y3], 'k-', lw=4)
    
    # 5. Trailer 2
    p_tl2_c = (np.array([x3, y3]) + np.array([x4, y4])) / 2
    draw_chassis(ax, p_tl2_c, trailer2_len, trailer2_width, theta4, color='blue', label='Trailer 2')
    
    draw_wheel_pair(ax, (x3, y3), theta3, W, color='black')
    draw_wheel_pair(ax, (x4, y4), theta4, W, color='black')
    
    ax.plot([x4, x_tail2], [y4, y_tail2], 'k-', lw=2) # Tail stub

    # Hitch Points
    ax.plot(xh1, yh1, 'ko', ms=6)
    ax.plot(xh2, yh2, 'ko', ms=6)

    # --- Annotations & Reference Lines ---
    
    # Centerlines
    ax.plot([x0, x0_f], [y0, y0_f], color='black', lw=4, alpha=0.4)
    ax.plot([x1, x2], [y1, y2], color='black', lw=4, alpha=0.4)
    ax.plot([x3, x4], [y3, y4], color='black', lw=4, alpha=0.4)

    # Reference Lines
    draw_dashed_ref(ax, (x0, y0)) 
    draw_dashed_ref(ax, (x2, y2)) 
    draw_dashed_ref(ax, (x4, y4))
    
    # Points
    ax.text(x0+0.05, y0+0.05, '$(x_0, y_0)$', color='blue')
    ax.text(x1+0.05, y1+0.05, '$(x_1, y_1)$', color='blue')
    ax.text(x2+0.05, y2+0.05, '$(x_2, y_2)$', color='red')
    ax.text(x3+0.05, y3+0.05, '$(x_3, y_3)$', color='blue')
    ax.text(x4+0.05, y4+0.05, '$(x_4, y_4)$', color='red')

    # Dimensions
    ax.annotate(f'$L_0={L0}$', xy=((x0+x0_f)/2, (y0+y0_f)/2), xytext=(-10, 10), textcoords='offset points')
    ax.annotate(f'$L_1={L1}$', xy=((xh1+x1)/2, (yh1+y1)/2), xytext=(-10, 10), textcoords='offset points')
    ax.annotate(f'$L_2={L2}$', xy=((x1+x2)/2, (y1+y2)/2), xytext=(-10, 10), textcoords='offset points')
    ax.annotate(f'$L_3={L3}$', xy=((xh2+x3)/2, (yh2+y3)/2), xytext=(-10, 10), textcoords='offset points')
    ax.annotate(f'$L_4={L4}$', xy=((x3+x4)/2, (y3+y4)/2), xytext=(-10, 10), textcoords='offset points')
    
    # Angles
    # Theta0
    patches.Arc((x0, y0), 3, 3, angle=0, theta1=0, theta2=np.degrees(theta0), color='blue')
    ax.text(x0+0.5, y0+0.1, r'$\theta_0$', color='blue')
    
    # Theta1
    patches.Arc((x1, y1), 3, 3, angle=0, theta1=np.degrees(theta2), theta2=np.degrees(theta1), color='green')
    ax.text(x1+0.5, y1, r'$\theta_1$', color='green')
    
    # Theta2
    patches.Arc((x2, y2), 3, 3, angle=0, theta1=0, theta2=np.degrees(theta2), color='red')
    ax.text(x2+0.5, y2+0.1, r'$\theta_2$', color='red')
    
    # Theta3
    patches.Arc((x3, y3), 3, 3, angle=0, theta1=np.degrees(theta4), theta2=np.degrees(theta3), color='green')
    ax.text(x3+0.5, y3, r'$\theta_3$', color='green')
    
    # Theta4
    patches.Arc((x4, y4), 3, 3, angle=0, theta1=0, theta2=np.degrees(theta4), color='red')
    ax.text(x4+0.5, y4+0.1, r'$\theta_4$', color='red')
    
    # Delta
    patches.Arc((x0_f, y0_f), 3, 3, angle=0, theta1=np.degrees(theta0), theta2=np.degrees(theta0+delta), color='orange')
    ax.text(x0_f+0.5, y0_f+0.5, r'$\delta$', color='orange')
    
    plt.title("Kinematic Diagram: Tractor + 2 Drawbar Trailers")
    plt.tight_layout()
    plt.savefig(save_path)
    print(f"Diagram saved to {save_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate Kinematic Diagram')
    parser.add_argument('--L0', type=float, default=2.0, help='Tractor Wheelbase')
    parser.add_argument('--L1', type=float, default=1.0, help='Drawbar 1 Length')
    parser.add_argument('--L2', type=float, default=1.2, help='Trailer 1 Wheelbase')
    parser.add_argument('--L3', type=float, default=1.0, help='Drawbar 2 Length')
    parser.add_argument('--L4', type=float, default=1.2, help='Trailer 2 Wheelbase')
    parser.add_argument('--W', type=float, default=1.0, help='Track Width')
    parser.add_argument('--tractor_width', type=float, default=1.5, help='Tractor Body Width')
    parser.add_argument('--tractor_len', type=float, default=2.8, help='Tractor Body Length')
    parser.add_argument('--trailer_width', type=float, default=1.5, help='Trailer Body Width')
    parser.add_argument('--trailer_len', type=float, default=2.0, help='Trailer Body Length')
    parser.add_argument('--tail_ext', type=float, default=0.15, help='Tail Extension')
    
    args = parser.parse_args()
    
    create_diagram(L0=args.L0, L1=args.L1, L2=args.L2, L3=args.L3, L4=args.L4, W=args.W, 
                   tractor_width=args.tractor_width, tractor_len=args.tractor_len,
                   trailer_width=args.trailer_width, trailer_len=args.trailer_len, 
                   tail_ext=args.tail_ext)
