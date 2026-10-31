"""Collect ORCA v2 joint chirp data through orca_core, without ROS.

By default, only validate and print the experiment plan. Pass --execute to
connect to a previously calibrated hand and collect real measurements.
"""

import argparse
import csv
from datetime import datetime, timezone
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy import signal
import yaml


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "src/hand_control/orca/configs/orcahand_v2/right/config.yaml"
)
CSV_FIELDS = ["timestamp", "commanded_position", "measured_position"]


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="This hand's config.yaml; calibration.yaml must be beside it.")
    parser.add_argument("--joints", nargs="+", default=["index_mcp"],
                        help="Joint names to sweep sequentially (ORCA v2 uses thumb_cmc).")
    parser.add_argument("--ctrl-freq", type=float, default=20.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--f0", type=float, default=0.1)
    parser.add_argument("--f1", type=float, default=5.0)
    parser.add_argument("--amplitude-deg", type=float, default=10.0)
    parser.add_argument("--center-deg", type=float, default=None,
                        help="Sweep center; by default use each joint's ROM midpoint.")
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--output-dir", type=Path, default=Path("logs/chirp_v2"))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="Connect and move real hardware.")
    mode.add_argument("--dry-run", action="store_true", help="Print the plan only (the default).")
    return parser


def make_plan(args, config):
    """Reject invalid or clipped experiments before constructing hardware."""
    for name in ("ctrl_freq", "duration", "f0", "f1", "amplitude_deg"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(args.settle_time) or args.settle_time < 0:
        raise ValueError("settle_time must be finite and non-negative")
    if not args.f0 < args.f1 < args.ctrl_freq / 2:
        raise ValueError("Require 0 < f0 < f1 < ctrl_freq / 2 (Nyquist frequency)")
    if args.duration * args.ctrl_freq < 2:
        raise ValueError("The experiment must contain at least two samples")
    if len(set(args.joints)) != len(args.joints):
        raise ValueError("Do not repeat joints in --joints")

    joint_ids = config["joint_ids"]
    if not joint_ids or len(set(joint_ids)) != len(joint_ids):
        raise ValueError("config joint_ids must be non-empty and unique")
    unknown = set(args.joints) - set(joint_ids)
    if unknown:
        raise ValueError(f"Unknown joints: {sorted(unknown)}. Available: {joint_ids}")
    neutral = config["neutral_position"]
    roms = config["joint_roms"]
    for joint in joint_ids:
        lower, upper = roms[joint]
        position = neutral[joint]
        if not all(math.isfinite(x) for x in (lower, upper, position)):
            raise ValueError(f"Non-finite ROM or neutral position for {joint}")
        if lower >= upper or not lower <= position <= upper:
            raise ValueError(f"Invalid ROM or neutral position for {joint}")

    centers = {}
    for joint in args.joints:
        lower, upper = roms[joint]
        center = (lower + upper) / 2 if args.center_deg is None else args.center_deg
        if (not math.isfinite(center)
                or center - args.amplitude_deg < lower
                or center + args.amplitude_deg > upper):
            raise ValueError(f"{joint}: center +/- amplitude exceeds ROM [{lower}, {upper}]")
        centers[joint] = center
    return {
        "config_path": str(args.config.resolve()),
        "config": config,
        "joints": args.joints,
        "centers_deg": centers,
        "ctrl_freq_hz": args.ctrl_freq,
        "duration_s": args.duration,
        "f0_hz": args.f0,
        "f1_hz": args.f1,
        "amplitude_deg": args.amplitude_deg,
        "settle_time_s": args.settle_time,
        "sample_count_per_joint": len(sample_times(args)),
    }


def sample_times(args):
    return np.arange(0, args.duration, 1.0 / args.ctrl_freq)


def collect_joint(hand, args, plan, joint, path, *, clock=time.monotonic, sleep=time.sleep):
    """Write each real sample immediately, preserving partial data on failure."""
    t = sample_times(args)
    trajectory = plan["centers_deg"][joint] + args.amplitude_deg * signal.chirp(
        t, f0=args.f0, t1=args.duration, f1=args.f1, method="logarithmic"
    )
    command = dict(plan["config"]["neutral_position"])
    command[joint] = float(trajectory[0])
    # Ramp for roughly one second rather than jumping to the first chirp sample.
    hand.set_joint_positions(command, num_steps=max(2, math.ceil(args.ctrl_freq)),
                             step_size=1.0 / args.ctrl_freq)
    sleep(args.settle_time)

    with path.open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        stream.flush()
        start = clock()
        for nominal_time, position in zip(t, trajectory):
            deadline = start + float(nominal_time)
            sleep(max(0.0, deadline - clock()))
            # Catching up in a burst would invalidate the intended excitation.
            if clock() - deadline >= 1.0 / args.ctrl_freq:
                raise RuntimeError("Missed a full control period; stopped instead of sending a burst. "
                                   "Check bus latency and the partial CSV timestamps.")
            command[joint] = float(position)
            hand.set_joint_positions(command)
            measured = hand.get_joint_position().as_dict().get(joint)
            if measured is None or not math.isfinite(measured):
                raise RuntimeError(f"No finite position measurement for {joint}")
            writer.writerow([clock() - start, float(position), float(measured)])
            stream.flush()
        # Hold the last sample for a full period before changing pose or disabling torque.
        sleep(max(0.0, start + len(t) / args.ctrl_freq - clock()))


def read_temperatures(hand):
    """Motor temperatures in degrees C by motor ID; None if the read fails, which must not stop a run."""
    try:
        return {str(motor): float(temp) for motor, temp in hand.get_motor_temp(as_dict=True).items()}
    except Exception:
        return None


def run_hardware(args, plan, *, factory=None, clock=time.monotonic, sleep=time.sleep, on_run_dir=None):
    """on_run_dir, if given, is called with the run directory as soon as it exists."""
    if factory is None:
        # Preview/help work without orca_core, ROS or any serial access.
        from orca_core import load_hand
        factory = load_hand

    hand = factory(config_path=str(args.config.resolve()))
    metadata = dict(plan, status="starting", csv_files=[])
    run_dir = None
    try:
        # init_joints() can auto-calibrate. Refuse to enter it without a complete
        # existing calibration: calibration is a separate, deliberate operation.
        if not hand.calibrated or not hand.is_calibrated():
            raise RuntimeError("A complete calibration.yaml for this hand is required beside config.yaml. "
                               "Calibrate separately before running chirp.")
        ok, message = hand.connect(interactive=False)
        if not ok:
            raise RuntimeError(f"Could not connect: {message}")
        print(message)

        started_at = datetime.now(timezone.utc)
        stamp = started_at.strftime("%Y%m%d_%H%M%S_%f")
        run_dir = args.output_dir / stamp
        run_dir.mkdir(parents=True, exist_ok=False)
        if on_run_dir is not None:
            on_run_dir(run_dir)
        metadata.update(status="running", orca_core_version=version("orca_core"),
                        run_id=stamp, started_at_utc=started_at.isoformat(),
                        motor_temperature_start_c=read_temperatures(hand))
        (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        hand.init_joints(force_calibrate=False, move_to_neutral=True)
        for joint in args.joints:
            filename = (f"chirp_{joint}_{stamp}_ctrl{args.ctrl_freq:g}Hz_"
                        f"{args.duration}s_{args.f0}-{args.f1}Hz_{args.amplitude_deg:g}deg.csv")
            path = run_dir / filename
            metadata["csv_files"].append(filename)
            print(f"Collecting {joint}: {path}")
            collect_joint(hand, args, plan, joint, path, clock=clock, sleep=sleep)
        metadata["status"] = "complete"
    except BaseException as exc:
        metadata.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "failed",
                        error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        # Always attempt both operations, including if connect/init fails.
        active_exception = sys.exc_info()[0] is not None
        cleanup_errors = []
        operations = []
        if hand.is_connected():
            metadata["motor_temperature_end_c"] = read_temperatures(hand)
            operations.append(("disable torque", hand.disable_torque))
        operations.append(("disconnect", hand.disconnect))
        for label, operation in operations:
            try:
                result = operation()
                if label == "disable torque" and result:
                    raise RuntimeError(f"Torque disable not acknowledged by motors {result}")
                if label == "disconnect" and result is not None and not result[0]:
                    raise RuntimeError(result[1])
            except Exception as exc:
                cleanup_errors.append(str(exc))
                print(f"Hardware cleanup ({label}) failed: {exc}", file=sys.stderr)
        if cleanup_errors:
            metadata.update(status="failed", cleanup_errors=cleanup_errors)
        if run_dir is not None:
            (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
            print(f"Experiment {metadata['status']}; data: {run_dir}")
        if cleanup_errors and not active_exception:
            raise RuntimeError("Hardware cleanup failed; check the hand connection")
    return run_dir


def main(argv=None):
    parser = make_parser()
    args = parser.parse_args(argv)
    try:
        with args.config.open() as stream:
            config = yaml.safe_load(stream)
        plan = make_plan(args, config)
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(json.dumps({key: value for key, value in plan.items() if key != "config"}, indent=2))
    if not args.execute:
        print("Preview only. Add --execute to connect and move the calibrated hand.")
        return 0
    try:
        run_hardware(args, plan)
    except KeyboardInterrupt:
        print("Stopped all sweeps; partial CSV data has been kept.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Chirp failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
