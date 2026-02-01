def lerp(a, b, t):
    return a + (b - a) * t


def build_joint_frames(results, frames_per_target=60, loop_blend=30):
    q = (0.0, 0.0)  # starting angles
    frames = []

    for block in results:
        if not block["reachable"]:
            continue
        if not block["solutions"]:
            continue

        # NEW: run every solution for this target in order
        for sol in block["solutions"]:
            q_next = (sol["theta1"], sol["theta2"])

            for i in range(frames_per_target):
                t = i / (frames_per_target - 1)
                frames.append((
                    lerp(q[0], q_next[0], t),
                    lerp(q[1], q_next[1], t)
                ))

            q = q_next

    # keep your smooth loop-back
    if frames and loop_blend > 0:
        start = frames[0]
        end = frames[-1]
        for i in range(1, loop_blend + 1):
            t = i / (loop_blend + 1)
            frames.append((
                lerp(end[0], start[0], t),
                lerp(end[1], start[1], t)
            ))

    return frames