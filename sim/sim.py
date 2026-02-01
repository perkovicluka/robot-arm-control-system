import argparse
from run_cpp_and_parse import run_and_parse
from sim_utils import build_joint_frames
from animate_arm import animate


def main():
    parser = argparse.ArgumentParser(
        description="Animate planar arm solutions and optionally export a GIF."
    )
    parser.add_argument("--exe", default="../arm_demo", help="Path to compiled C++ demo executable")
    parser.add_argument("--gif", metavar="PATH", help="Save animation as GIF to PATH (e.g., output.gif)")
    parser.add_argument("--frames-per-target", type=int, default=60, help="Interpolation frames per target")
    parser.add_argument("--fps", type=float, help="FPS for saved GIF (default matches live interval)")
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=16,
        help="Frame delay for live animation (ms). Also used to derive GIF fps if --fps not set.",
    )
    args = parser.parse_args()

    results = run_and_parse(args.exe)
    frames = build_joint_frames(results, frames_per_target=args.frames_per_target)
    anim = animate(
        frames,
        L1=1.0,
        L2=1.0,
        segment_frames=args.frames_per_target,
        save_path=args.gif,
        interval_ms=args.interval_ms,
        fps=args.fps,
    )
    return anim


if __name__ == "__main__":
    main()
