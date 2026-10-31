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

## Record all joints

`chirp_all.py` records all 17 joints in one run (about 6 minutes), so you can
fit one finger and use the other three to find outliers and measure the spread:

```bash
python experiments/orca/chirp_all.py            # plan only
python experiments/orca/chirp_all.py --execute  # move the hand
```

It uses the installed `~/ros_ws/install/.../right/config.yaml` and the
`calibration.yaml` beside it; pass `--config` for another workspace. You do not
need the copy from step 2 above. The sweep options are the same as for `chirp.py`, and
`--joints` limits the run to some joints.

`--finger` (default `middle`) only saves the finger meant for the fit in
`metadata.json`, under `fit_finger` and `fit_joints`, so you can still fit another finger later.
Before it moves, the script warns you if a joint's motor travel in `calibration.yaml` is more than
25% below the median of the same joint on all fingers, and asks whether to continue.

After the run it prints a table, opens `overview.png`, and asks whether the run
was good and for optional notes (or pass `--notes`). The answer is added to the folder name:

| Suffix | Meaning |
|---|---|
| `_good` | You answered `y` |
| `_bad` | You answered `n`: the run itself was disturbed. A genuine outlier is still a good run |
| `_untagged` | You pressed Enter or Ctrl+C at the prompt; rename the folder when you decide |
| `_interrupted` | You stopped the run with Ctrl+C; the partial data is kept |
| `_failed` | The run stopped with an error; the partial data is kept |

The table flags `low motion` (moved less than 75% of the commanded peak-to-peak range), `short travel`,
`too few samples`, `timing` and `no data`. `overview.png` shows each joint type in one column.

## Collect data

To record only some joints, use `chirp.py`. The minimum for whole-hand SysID is eight representative
joints: the wrist, four thumb joints, and three joints of one finger. faive_lab fits these eight
and reuses that finger's parameters for the other three fingers (nine more joints). This reduces
recording and fitting time; it assumes similar dynamics, not identical measured behavior.

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

`chirp_all.py` adds the suffix above to the folder, and also saves `config.yaml`, `calibration.yaml` and
`overview.png` in it. The timestamp in the folder name is in UTC. `metadata.json` includes the motor
temperatures at the start and end of the run.

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
