"""Record chirps on all ORCA v2 joints for one SysID run, without ROS.

By default, only validate and print the experiment plan. Pass --execute to
move the hand. The run folder keeps the config and calibration it used. After
the run, it shows a quick check of the data, asks whether the run was good, and
adds the answer to the folder name:

    logs/chirp_v2/<timestamp>_good   (or _bad, _untagged, _failed, _interrupted)

--finger only records which finger is meant for the fit; every run records all joints.
"""

from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

import chirp


INSTALLED_CONFIG = (
    Path.home() / "ros_ws/install/hand_control/share/hand_control/orca/configs/orcahand_v2/right/config.yaml"
)
WRIST_AND_THUMB = ["wrist", "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]
FINGERS = ["index", "middle", "ring", "pinky"]
FINGER_JOINTS = ["abd", "mcp", "pip"]
# orca_core's default margin for a joint's motor travel (calibration_travel_margin).
SHORTFALL_MARGIN = 0.25
# Mean sample period may differ this much from the commanded one.
TIMING_MARGIN = 0.05


def make_parser():
    parser = chirp.make_parser()
    parser.description = __doc__
    parser.set_defaults(config=INSTALLED_CONFIG, joints=None)
    parser.add_argument("--finger", choices=FINGERS, default="middle",
                        help="Finger meant for the fit, saved in metadata.json. All fingers are recorded.")
    parser.add_argument("--notes", default="",
                        help="Saved in metadata.json, e.g. 're-tensioned index'. Asked after the run if empty.")
    return parser


def default_joints(finger):
    """Fit joints first, so a stopped run still has them."""
    fingers = [finger] + [other for other in FINGERS if other != finger]
    return WRIST_AND_THUMB + [f"{name}_{joint}" for name in fingers for joint in FINGER_JOINTS]


def short_travel(calibration):
    """{joint: (travel, median of the same joint on all fingers)} for joints far below that median."""
    travel = calibration.get("motor_travel_measured") or {}
    short = {}
    for kind in FINGER_JOINTS:
        group = {f"{name}_{kind}": travel.get(f"{name}_{kind}") for name in FINGERS}
        values = [value for value in group.values() if value is not None]
        if not values:
            continue
        median = float(np.median(values))
        short.update({joint: (value, median) for joint, value in group.items()
                      if value is not None and value < (1 - SHORTFALL_MARGIN) * median})
    return short


def ask(prompt):
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return ""


def read_recording(path):
    """Rows of (timestamp, commanded, measured), or None if there is no data."""
    if not path.exists() or path.stat().st_size == 0:
        return None
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return data if data.size else None


def summarize(run_dir, metadata, short):
    summary = {}
    period = 1.0 / metadata["ctrl_freq_hz"]
    for joint, filename in zip(metadata["joints"], metadata["csv_files"]):
        data = read_recording(run_dir / filename)
        if data is None:
            summary[joint] = dict(data=None, samples=0, moved=0.0, period_ms=float("nan"), flags=["no data"])
            continue
        commanded, measured = np.ptp(data[:, 1]), np.ptp(data[:, 2])
        moved = measured / commanded if commanded > 0 else 0.0
        mean_period = float(np.diff(data[:, 0]).mean()) if len(data) > 1 else float("nan")
        flags = []
        if len(data) < metadata["sample_count_per_joint"]:
            flags.append("too few samples")
        if moved < 1 - SHORTFALL_MARGIN:
            flags.append("low motion")
        if len(data) > 1 and abs(mean_period - period) > TIMING_MARGIN * period:
            flags.append("timing")
        if joint in short:
            flags.append("short travel")
        summary[joint] = dict(data=data, samples=len(data), moved=moved, period_ms=mean_period * 1e3, flags=flags)
    return summary


def print_summary(summary, calibration):
    travel = calibration.get("motor_travel_measured") or {}
    print(f"\n{'joint':<12}{'samples':>8}{'moved':>8}{'period':>10}{'travel':>9}  flags")
    for joint, row in summary.items():
        travel_text = f"{travel[joint]:.1f}" if joint in travel else "-"
        print(f"{joint:<12}{row['samples']:>8}{row['moved']:>8.0%}{row['period_ms']:>7.1f} ms"
              f"{travel_text:>9}  {', '.join(row['flags'])}")
    print("moved: measured / commanded peak-to-peak. travel: motor travel in calibration.yaml, in degrees.\n")


def write_overview(run_dir, metadata, summary):
    """Commanded and measured motion, with each joint type in one column. Returns the path, or None."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Skipping overview.png: matplotlib is not installed.")
        return None
    rows = [WRIST_AND_THUMB] + [[f"{name}_{joint}" for joint in FINGER_JOINTS] for name in FINGERS]
    if not any(row["data"] is not None for row in summary.values()):
        return None
    figure, axes = plt.subplots(len(rows), len(WRIST_AND_THUMB), figsize=(18, 11), sharex=True, sharey=True)
    fit_prefix = f"{metadata.get('fit_finger')}_"
    for axes_row, joints in zip(axes, rows):
        for column, axis in enumerate(axes_row):
            joint = joints[column] if column < len(joints) else None
            row = summary.get(joint)
            if row is None or row["data"] is None:
                axis.axis("off")
                continue
            center = metadata["centers_deg"][joint]
            axis.plot(row["data"][:, 0], row["data"][:, 1] - center, lw=.6, color="0.7", label="commanded")
            axis.plot(row["data"][:, 0], row["data"][:, 2] - center, lw=.6, color="tab:blue", label="measured")
            label = f"{joint}{' (fit)' if joint.startswith(fit_prefix) else ''}: {row['moved']:.0%}"
            axis.set_title(label, fontsize=10, color="tab:red" if row["flags"] else "black")
            axis.tick_params(labelbottom=True, labelleft=True)
    figure.supxlabel("time [s]")
    figure.supylabel("angle about the sweep center [deg]")
    figure.suptitle(f"{run_dir.name}: commanded (grey) and measured (blue); red titles have flags")
    figure.tight_layout()
    path = run_dir / "overview.png"
    figure.savefig(path, dpi=110)
    plt.close(figure)
    return path


def open_image(path):
    """Best effort: on a machine without a display, the path printed by the caller is enough."""
    command = {"darwin": "open", "linux": "xdg-open"}.get(sys.platform)
    if command and shutil.which(command):
        subprocess.Popen([command, str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def finish_run(run_dir, args, calibration):
    """Check the data, tag the run and rename its folder. Returns the new path."""
    metadata_path = run_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    try:
        summary = summarize(run_dir, metadata, short_travel(calibration))
        print_summary(summary, calibration)
        overview = write_overview(run_dir, metadata, summary)
        if overview is not None:
            print(f"Overview: {overview}")
            open_image(overview)
    except Exception as exc:
        # The recording is already safe; a failed check must not cost the tag.
        print(f"Could not check the data: {exc}", file=sys.stderr)

    notes = args.notes
    if metadata["status"] == "complete":
        print("Answer n only if the run itself was disturbed (bumped, loaded, loose cable). "
              "A genuine outlier is still a good run.")
        answer = ask("Good run? [y/n, Enter to decide later] ").lower()[:1]
        tag = {"y": "good", "n": "bad"}.get(answer, "untagged")
        notes = notes or ask("Notes (optional): ")
    else:
        tag = "interrupted" if metadata["status"] == "interrupted" else "failed"
    metadata.update(tag=tag, notes=notes)
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    tagged = run_dir.with_name(f"{run_dir.name}_{tag}")
    run_dir.rename(tagged)
    print(f"Run saved as {tagged}")
    return tagged


def main(argv=None, *, factory=None, clock=time.monotonic, sleep=time.sleep):
    parser = make_parser()
    args = parser.parse_args(argv)
    args.joints = args.joints or default_joints(args.finger)
    source_config = args.config
    source_calibration = source_config.parent / "calibration.yaml"
    try:
        config = yaml.safe_load(source_config.read_text())
        calibration = yaml.safe_load(source_calibration.read_text()) or {}
        plan = chirp.make_plan(args, config)
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        parser.error(f"{exc}. Calibrate first, or pass your config with --config.")
    modified = datetime.fromtimestamp(source_calibration.stat().st_mtime, timezone.utc)
    plan.update(config_path=str(source_config), fit_finger=args.finger,
                fit_joints=[joint for joint in args.joints
                            if joint in WRIST_AND_THUMB or joint.startswith(f"{args.finger}_")],
                calibration_modified_at_utc=modified.isoformat())

    print(json.dumps({key: value for key, value in plan.items() if key != "config"}, indent=2))
    minutes = len(args.joints) * (args.duration + args.settle_time + 1) / 60
    print(f"About {minutes:.0f} min for {len(args.joints)} joints.")
    short = short_travel(calibration)
    for joint, (travel, median) in short.items():
        print(f"WARNING: {joint} has {travel:.1f} deg of motor travel, but the median of its group is "
              f"{median:.1f} deg. It probably stalled in calibration; see Troubleshooting in experiments/README.md.")
    if not args.execute:
        print("Preview only. Add --execute to connect and move the calibrated hand.")
        return 0
    if short and ask("Record anyway? [y/N] ").lower()[:1] != "y":
        print("Nothing was recorded.")
        return 1

    run_dirs = []

    def on_run_dir(run_dir):
        run_dirs.append(run_dir)
        shutil.copy2(source_config, run_dir / "config.yaml")
        shutil.copy2(source_calibration, run_dir / "calibration.yaml")

    code = 0
    # chirp.run_hardware looks for calibration.yaml beside the resolved config. An installed
    # config can be a symlink into the source folder, so run from real copies of both files.
    with tempfile.TemporaryDirectory() as snapshot:
        args.config = Path(snapshot) / "config.yaml"
        shutil.copy2(source_config, args.config)
        shutil.copy2(source_calibration, Path(snapshot) / "calibration.yaml")
        try:
            chirp.run_hardware(args, plan, factory=factory, clock=clock, sleep=sleep, on_run_dir=on_run_dir)
        except KeyboardInterrupt:
            print("Stopped all sweeps; partial CSV data has been kept.", file=sys.stderr)
            code = 130
        except Exception as exc:
            print(f"Chirp failed: {exc}", file=sys.stderr)
            code = 1
    if run_dirs:
        finish_run(run_dirs[0], args, calibration)
    return code


if __name__ == "__main__":
    sys.exit(main())
