#!/usr/bin/env python3
"""Inspect raw or synchronized demonstration HDF5 files without robot hardware."""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np


MOTION = {
    "/right/arm/end_effector_pose": ("observations/right/qpos_arm", 7),
    "/right/arm/end_effector_pose_cmd": ("right/actions_arm", 7),
    "/right/hand/joint_position_from_motor": ("observations/right/qpos_hand", 17),
    "/right/hand/policy_output": ("right/actions_hand", 17),
}


def inspect(path, cameras, output=None, max_gap=0.5):
    report = {"file": str(path), "streams": {}, "errors": []}
    with h5py.File(path, "r") as handle:
        raw = "observations" not in handle
        report["format"] = "raw" if raw else "synchronized"
        specs = [(topic, key, size) for topic, (key, size) in MOTION.items()]
        specs += [(f"/{cam}/color", f"observations/images/{cam}/color", None) for cam in cameras]
        streams = {}
        for topic, key, size in specs:
            selected = topic if raw else key
            if selected not in handle:
                report["errors"].append(f"Missing required stream: {selected}")
                continue
            source = handle[selected]
            if raw:
                stamps = np.array(sorted(int(k) for k in source), dtype=np.int64)
                times = stamps / 1e9
                get = lambda i, s=source, t=stamps: s[str(t[i])][()]
            else:
                count = len(source)
                if "timestamps" in handle:
                    times = handle["timestamps"][:] / 1e9
                    if len(times) != count:
                        report["errors"].append(f"Length mismatch: {selected}")
                        continue
                else:
                    times = np.arange(count) / float(handle.attrs.get("sampling_frequency", 10))
                get = lambda i, s=source: s[i]
            if len(times) < 2:
                report["errors"].append(f"Fewer than two samples: {selected}")
                continue
            gaps = np.diff(times)
            if np.any(gaps <= 0) or gaps.max() > max_gap:
                report["errors"].append(f"Non-increasing timestamps or gap above {max_gap}s: {selected}")
            first = np.asarray(get(0))
            stats = {"samples": len(times), "sample_shape": list(first.shape),
                     "duration_s": float(times[-1] - times[0]), "max_gap_s": float(gaps.max())}
            if size is not None:
                values = np.array([get(i) for i in range(len(times))])
                if values.shape != (len(times), size) or not np.isfinite(values).all():
                    report["errors"].append(f"Invalid shape or non-finite values: {selected}")
                elif size == 7 and not np.allclose(np.linalg.norm(values[:, 3:], axis=1), 1, atol=0.05):
                    report["errors"].append(f"Invalid quaternion norms: {selected}")
                stats["range"] = np.ptp(values, axis=0).tolist()
            elif first.ndim != 3 or first.shape[-1] != 3:
                report["errors"].append(f"Expected HxWx3 image: {selected}")
            report["streams"][topic] = stats
            streams[topic] = (times, get)
        if len(streams) == len(specs):
            begin = max(t[0] for t, _ in streams.values())
            end = min(t[-1] for t, _ in streams.values())
            report["overlap_s"] = float(max(0, end - begin))
            if end <= begin:
                report["errors"].append("Required streams have no overlapping time interval")
            if output is not None and not report["errors"]:
                output = Path(output)
                output.mkdir(parents=True, exist_ok=True)
                _preview(streams, cameras, begin, end, output)
        report["ok"] = not report["errors"]
    if output is not None:
        Path(output).mkdir(parents=True, exist_ok=True)
        (Path(output) / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def _preview(streams, cameras, begin, end, output):
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    writer = cv2.VideoWriter(str(output / "preview.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (320 * len(cameras), 180))
    if not writer.isOpened():
        raise RuntimeError("Could not open preview video writer")
    try:
        for stamp in np.arange(begin, end, 0.1):
            frames = []
            for cam in cameras:
                times, get = streams[f"/{cam}/color"]
                frame = cv2.resize(get(int(np.argmin(np.abs(times - stamp)))), (320, 180))
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.putText(frame, cam, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                frames.append(frame)
            writer.write(np.concatenate(frames, axis=1))
    finally:
        writer.release()
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    for topic, (times, get) in streams.items():
        if topic in MOTION:
            values = np.array([get(i) for i in range(len(times))])
            arm = MOTION[topic][1] == 7
            axes[0 if arm else 1].plot(times - begin, values[:, :3] if arm else values,
                                      linestyle="--" if "cmd" in topic or "policy_output" in topic else "-")
    axes[0].set_ylabel("Arm xyz (m)")
    axes[1].set_ylabel("Hand joints (rad)")
    axes[1].set_xlabel("Seconds from shared interval start; dashed = command")
    fig.tight_layout()
    fig.savefig(output / "trajectory.png", dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--cameras", nargs="+", default=["oakd_front_view", "oakd_wrist_view"])
    parser.add_argument("--output", type=Path, help="Directory for summary, preview video and trajectory plot")
    parser.add_argument("--max-gap", type=float, default=0.5)
    args = parser.parse_args()
    report = inspect(args.recording, args.cameras, args.output, args.max_gap)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
