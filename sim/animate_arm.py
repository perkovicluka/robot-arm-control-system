import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arc, Circle


def fk(L1, L2, theta1, theta2):
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)
    return (0.0, 0.0), (x1, y1), (x2, y2)


def rad_to_deg(r):
    return r * 180.0 / math.pi


def animate(
    frames,
    L1=1.0,
    L2=1.0,
    segment_frames=60,
    all_targets=None,
    save_path=None,
    interval_ms=16,
    fps=None,
):
    fig, ax = plt.subplots()
    ax.set_aspect("equal", "box")

    reach = L1 + L2 + 0.2
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)

    # Workspace reachability rings
    outer_r = L1 + L2
    inner_r = abs(L1 - L2)

    ax.add_patch(Circle((0.0, 0.0), outer_r, fill=False, linestyle="--", linewidth=1.0))
    if inner_r > 1e-9:
        ax.add_patch(Circle((0.0, 0.0), inner_r, fill=False, linestyle="--", linewidth=1.0))

    # Arm line
    line, = ax.plot([], [], marker="o")

    # Segment endpoints become targets. segment_frames must match how frames were generated.
    if segment_frames < 1:
        segment_frames = 1

    target_indices = list(range(segment_frames - 1, len(frames), segment_frames))
    targets = []
    for idx in target_indices:
        t1, t2 = frames[idx]
        _, _, tip = fk(L1, L2, t1, t2)
        targets.append(tip)

    current_target_idx = 0

    # Target point (current goal)
    target_point, = ax.plot([], [], "ro", markersize=6)

    # All requested targets: reachable as faint dots, unreachable as gray X
    all_reachable, = ax.plot([], [], "ko", markersize=4, alpha=0.25)
    all_unreachable, = ax.plot([], [], "x", markersize=7, alpha=0.6)

    def is_reachable(x, y):
        r = math.hypot(x, y)
        return (r >= abs(L1 - L2) - 1e-9) and (r <= (L1 + L2) + 1e-9)

    # If the caller provides all_targets, classify and plot them.
    # Otherwise, fall back to plotting the reachable segment-end targets we inferred from frames.
    if all_targets is None:
        all_targets = []
        for pt in targets:
            all_targets.append((pt[0], pt[1]))

    rx, ry, ux, uy = [], [], [], []
    for x, y in all_targets:
        if is_reachable(x, y):
            rx.append(x)
            ry.append(y)
        else:
            ux.append(x)
            uy.append(y)

    all_reachable.set_data(rx, ry)
    all_unreachable.set_data(ux, uy)

    # Text overlay
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    # Angle value labels (placed near the arcs)
    theta1_label = ax.text(0.0, 0.0, "", ha="center", va="center")
    theta2_label = ax.text(0.0, 0.0, "", ha="center", va="center")

    # Angle arcs
    theta1_arc = Arc((0, 0), 0.35, 0.35, angle=0, theta1=0, theta2=0, color="tab:blue")
    theta2_arc = Arc((0, 0), 0.30, 0.30, angle=0, theta1=0, theta2=0, color="tab:orange")

    ax.add_patch(theta1_arc)
    ax.add_patch(theta2_arc)

    def update(i):
        t1, t2 = frames[i]

        nonlocal current_target_idx
        # FuncAnimation repeats by restarting frame index at 0. Reset target state on loop.
        if i == 0:
            current_target_idx = 0

        if targets:
            tx, ty = targets[current_target_idx]
            target_point.set_data([tx], [ty])

            # When we finish the current segment, move to the next target
            if current_target_idx < len(target_indices) - 1:
                if i >= target_indices[current_target_idx]:
                    current_target_idx += 1

        p0, p1, p2 = fk(L1, L2, t1, t2)

        # Update arm
        line.set_data(
            [p0[0], p1[0], p2[0]],
            [p0[1], p1[1], p2[1]]
        )

        # --- θ1 arc (base) ---
        theta1_arc.center = p0
        theta1_arc.theta1 = 0.0
        theta1_arc.theta2 = rad_to_deg(t1)

        # --- θ2 arc (elbow, relative joint angle between link1 and link2) ---
        # Arc in matplotlib is drawn CCW from theta1 -> theta2. To visualize the *relative* angle
        # correctly (including negative angles), draw the smallest arc of magnitude |theta2|
        # starting from the link1 direction.
        theta2_arc.center = p1
        base_deg = rad_to_deg(t1)
        rel_deg = rad_to_deg(t2)

        if rel_deg >= 0:
            theta2_arc.theta1 = base_deg
            theta2_arc.theta2 = base_deg + rel_deg
        else:
            theta2_arc.theta1 = base_deg + rel_deg
            theta2_arc.theta2 = base_deg

        # Place and update angle labels near the middle of each arc
        r1 = 0.18  # label radius for theta1 arc
        r2 = 0.16  # label radius for theta2 arc

        mid1 = t1 * 0.5
        theta1_label.set_position((p0[0] + r1 * math.cos(mid1), p0[1] + r1 * math.sin(mid1)))
        theta1_label.set_text(f"θ1={rad_to_deg(t1):.1f}°")

        # Place label in the middle of the drawn theta2 arc
        mid2 = t1 + 0.5 * t2
        theta2_label.set_position((p1[0] + r2 * math.cos(mid2), p1[1] + r2 * math.sin(mid2)))
        theta2_label.set_text(f"θ2={rad_to_deg(t2):.1f}°")

        # Text overlay
        info.set_text(
            f"frame {i}/{len(frames)-1}\n"
            f"tip = ({p2[0]:.3f}, {p2[1]:.3f})"
        )

        return (line, target_point, all_reachable, all_unreachable, info, theta1_arc, theta2_arc, theta1_label, theta2_label)

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frames),
        interval=interval_ms,
        blit=False,
        repeat=True
    )

    # If a path is provided, write a GIF instead of (or in addition to) showing the window.
    if save_path:
        # If caller didn't set fps, derive it from the live-play interval so speeds match.
        effective_fps = fps if fps is not None else (1000.0 / interval_ms)
        anim.save(save_path, writer="pillow", fps=effective_fps)
        plt.close(fig)
    else:
        plt.show()

    return anim
