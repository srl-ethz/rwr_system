# ORCA v2 chirp and joint data acquisition

`chirp.py` collects ORCA v2 data through `orca_core`'s
public joint-position API. It runs directly with Python on macOS/Linux.

## Install for standalone v2 acquisition

From the repository root, create a separate Python 3.10+ environment (for
example, Python 3.12):

```bash
python3.12 -m venv .venv.chirp
source .venv.chirp/bin/activate
python -m pip install orca_core==0.5.2 scipy
```

No editable installation of `faive_system` is necessary for this standalone
script. A separate environment avoids the conflict between `orca_core`'s
NumPy requirement and `faive_system`'s NumPy 1.26.4 pin. For an existing
Faive/ROS environment, see the installation notes in
[`src/hand_control/README.md`](../../src/hand_control/README.md).

On macOS, you can use the `rwr_jazzy` environment instead (see the
[main README](../../README.md)). It already contains `orca_core` and `scipy`.

## Prepare the hand

1. Tension and calibrate the correct physical hand using the existing v2
   workflow. Chirp requires complete calibration and will **not** automatically
   calibrate it. See the hand-control README above.
2. Prepare your hand's own `config.yaml` and place its `calibration.yaml`
   in the same directory. Pass your config path with `--config` in the
   commands below, replacing `/path/to/your/config.yaml` with your file.
   After a `colcon build --symlink-install`, the installed `config.yaml` is a
   symlink to the source folder, and `chirp.py` then looks for `calibration.yaml`
   in the source folder. Copy both files to a folder of their own and use that:
   ```bash
   mkdir -p ~/ros_ws/chirp_config
   cp -L ~/ros_ws/install/hand_control/share/hand_control/orca/configs/orcahand_v2/right/{config,calibration}.yaml ~/ros_ws/chirp_config/
   ```
   Copy them again after each new calibration or change of `config.yaml`.
3. Connect power and USB. Keep `port: auto` in your config for automatic
   detection. If it fails or selects the wrong device, set `port` manually
   to the actual serial device (see Troubleshooting).
4. Stop the ROS controller, GUI, tensioning tool, and any other program using
   the same serial port. Leave the hand unloaded with room to move.

## Collect data

For whole-hand SysID, collect eight representative joints: the wrist,
four thumb joints, and three index-finger joints. faive_lab fits these eight
and reuses each index-finger joint's parameters for the corresponding
middle, ring, and pinky joints (nine more joints). This reduces recording
and fitting time; it assumes similar dynamics, not identical measured behavior.

```bash
python experiments/orca/chirp.py \
  --config /path/to/your/config.yaml \
  --joints wrist thumb_cmc thumb_abd thumb_mcp thumb_dip index_abd index_mcp index_pip \
  --ctrl-freq 20 --duration 20 --f0 0.1 --f1 5 --amplitude-deg 10 --execute
```

Each joint runs a 20-second logarithmic sweep from 0.1 to 5 Hz at 20 Hz
control, with ±10° about its ROM midpoint. Other joints hold their neutral
pose. Use `--center-deg` to change the center; the trajectory must fit within
the joint limits. Ctrl+C stops the experiment.

## Output

Each run creates a directory under `logs/chirp_v2/` (`--output-dir` to change):

```text
logs/chirp_v2/<timestamp>/
├── metadata.json
└── chirp_<joint>_*.csv   # one CSV per selected joint
```

CSV columns (seconds and degrees; 400 rows per joint with the settings above):

```text
timestamp,commanded_position,measured_position
```

Keep the original filenames and `metadata.json` together. Only use recordings
marked `complete` in metadata for SysID. On motor-only hands, measured angles
are calibrated motor-derived estimates, not independent encoder measurements.

## Inspect a recording

Plot the commanded/measured angles and Bode response:

```bash
python -m pip install pandas matplotlib
python experiments/orca/plot.py /path/to/recording.csv --save
```

The PNG is saved in the current directory. On a headless machine, prefix
the command with `MPLBACKEND=Agg`. Plotting does not change the CSVs or SysID
settings; see `plot.py --help` for trim and alignment options.

## Use the recording in faive_lab SysID

[faive_lab SysID guide](https://github.com/srl-ethz/faive_lab/blob/rwr2026/scripts/sysid/README.md)

For the fit commands, see "The hand" in the [main README](../../README.md).

## Troubleshooting

**Serial connection:** list devices, then set `port` in your config if auto
detection fails:

```bash
python -m serial.tools.list_ports
```

**Check the plan without motion:**

```bash
python experiments/orca/chirp.py \
  --config /path/to/your/config.yaml --joints index_mcp --dry-run
```

**Check one joint with a small amplitude:**

```bash
python experiments/orca/chirp.py \
  --config /path/to/your/config.yaml \
  --joints index_mcp --amplitude-deg 3 --execute
```
