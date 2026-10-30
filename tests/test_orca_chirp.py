"""Offline acquisition tests; never open serial ports or move a physical hand."""

import csv
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "orca_chirp", ROOT / "experiments/orca/chirp.py"
)
chirp = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(chirp)
RENAME_SPEC = importlib.util.spec_from_file_location(
    "rename_chirp", ROOT / "experiments/orca/rename_chirp_results.py"
)
rename_chirp = importlib.util.module_from_spec(RENAME_SPEC)
RENAME_SPEC.loader.exec_module(rename_chirp)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class Hand:
    calibrated = True

    def __init__(self, clock):
        self.clock = clock
        self.commands = []
        self.events = []
        self.reads = 0
        self.read_latency = 0.0
        self.interrupt_on_read = None
        self.fail_init = False
        self.fail_disable = False
        self.connected = False
        self.unacknowledged_motors = []
        self.disconnect_result = (True, "Disconnected")

    def is_calibrated(self):
        return self.calibrated

    def connect(self, **kwargs):
        self.events.append("connect")
        self.connected = True
        return True, "Fake connection"

    def is_connected(self):
        return self.connected

    def init_joints(self, **kwargs):
        assert kwargs == dict(force_calibrate=False, move_to_neutral=True)
        self.events.append("init")
        if self.fail_init:
            raise RuntimeError("initialization failed")

    def set_joint_positions(self, command, **kwargs):
        self.commands.append(dict(command))

    def get_joint_position(self):
        self.reads += 1
        if self.reads == self.interrupt_on_read:
            raise KeyboardInterrupt()
        self.clock.sleep(self.read_latency)
        measured = {joint: value - 0.25 for joint, value in self.commands[-1].items()}
        return SimpleNamespace(as_dict=lambda: measured)

    def disable_torque(self):
        self.events.append("disable")
        if self.fail_disable:
            raise RuntimeError("disable failed")
        return self.unacknowledged_motors

    def disconnect(self):
        self.events.append("disconnect")
        self.connected = False
        return self.disconnect_result


@pytest.fixture
def experiment(tmp_path, monkeypatch):
    args = chirp.make_parser().parse_args([
        "--duration", "0.2", "--settle-time", "0", "--output-dir", str(tmp_path),
    ])
    config = yaml.safe_load(args.config.read_text())
    plan = chirp.make_plan(args, config)
    clock = Clock()
    hand = Hand(clock)
    monkeypatch.setattr(chirp, "version", lambda name: "test")
    return args, plan, clock, hand


def run(experiment):
    args, plan, clock, hand = experiment
    return chirp.run_hardware(args, plan, factory=lambda **kwargs: hand,
                             clock=clock, sleep=clock.sleep)


def csv_rows(run_dir):
    with next(run_dir.glob("*.csv")).open() as stream:
        return list(csv.DictReader(stream))


def test_preview_never_imports_hardware_or_creates_output(tmp_path, monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "orca_core", None)
    assert chirp.main(["--joints", "thumb_cmc", "wrist", "--output-dir", str(tmp_path)]) == 0
    assert "Preview only" in capsys.readouterr().out
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("options", [
    ["--joints", "thumb_pip"],  # v1 name does not exist in v2
    ["--joints", "wrist", "wrist"],
    ["--ctrl-freq", "10"],  # f1 must be strictly below Nyquist
    ["--duration", "0"],
    ["--duration", "nan"],
    ["--amplitude-deg", "1000"],
    ["--center-deg", "nan"],
    ["--center-deg", "100"],
    ["--settle-time", "-1"],
])
def test_invalid_plan_is_rejected_without_hardware(options, monkeypatch):
    monkeypatch.setitem(sys.modules, "orca_core", None)
    with pytest.raises(SystemExit) as exc:
        chirp.main(options + ["--execute"])
    assert exc.value.code == 2


def test_real_samples_keep_csv_schema_and_other_joints_neutral(experiment):
    args, plan, clock, hand = experiment
    run_dir = run(experiment)
    csv_path, = run_dir.glob("chirp_index_mcp_*.csv")
    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["csv_files"] == [csv_path.name]
    assert csv_path.name == (
        f"chirp_index_mcp_{run_dir.name}_ctrl{args.ctrl_freq:g}Hz_"
        f"{args.duration}s_{args.f0}-{args.f1}Hz_{args.amplitude_deg:g}deg.csv"
    )
    assert metadata["run_id"] == run_dir.name
    assert metadata["started_at_utc"].endswith("+00:00")
    rows = csv_rows(run_dir)
    assert list(rows[0]) == chirp.CSV_FIELDS
    assert len(rows) == 4
    np.testing.assert_allclose([float(row["timestamp"]) for row in rows], [0, .05, .1, .15])
    for row in rows:
        assert float(row["measured_position"]) == pytest.approx(float(row["commanded_position"]) - .25)
    for command in hand.commands:
        for joint, neutral in plan["config"]["neutral_position"].items():
            if joint != "index_mcp":
                assert command[joint] == neutral
    assert hand.events == ["connect", "init", "disable", "disconnect"]
    assert clock.now == pytest.approx(.2)
    assert json.loads((run_dir / "metadata.json").read_text())["status"] == "complete"


def test_uncalibrated_hand_does_not_connect_or_auto_calibrate(experiment):
    args, _, _, hand = experiment
    hand.calibrated = False
    with pytest.raises(RuntimeError, match="complete calibration"):
        run(experiment)
    assert hand.events == ["disconnect"]
    assert not hand.commands
    assert not list(args.output_dir.iterdir())


def test_interrupt_preserves_partial_csv_and_skips_remaining_joints(experiment):
    args, _, _, hand = experiment
    args.joints = ["index_mcp", "thumb_cmc"]
    experiment = (args, chirp.make_plan(args, experiment[1]["config"]), *experiment[2:])
    hand.interrupt_on_read = 2
    with pytest.raises(KeyboardInterrupt):
        run(experiment)
    run_dir = next(args.output_dir.iterdir())
    assert len(csv_rows(run_dir)) == 1
    assert len(list(run_dir.glob("*.csv"))) == 1
    assert hand.events[-2:] == ["disable", "disconnect"]
    assert json.loads((run_dir / "metadata.json").read_text())["status"] == "interrupted"


def test_failed_initialization_still_cleans_up_and_records_failure(experiment):
    args, _, _, hand = experiment
    hand.fail_init = True
    with pytest.raises(RuntimeError, match="initialization failed"):
        run(experiment)
    assert hand.events[-2:] == ["disable", "disconnect"]
    metadata = json.loads(next(args.output_dir.glob("*/metadata.json")).read_text())
    assert metadata["status"] == "failed"


def test_failed_torque_disable_still_disconnects_and_returns_error(experiment):
    args, _, _, hand = experiment
    hand.fail_disable = True
    with pytest.raises(RuntimeError, match="cleanup failed"):
        run(experiment)
    assert hand.events[-2:] == ["disable", "disconnect"]
    metadata = json.loads(next(args.output_dir.glob("*/metadata.json")).read_text())
    assert metadata["status"] == "failed"
    assert metadata["cleanup_errors"]


def test_slow_bus_stops_without_catchup_burst(experiment):
    args, _, _, hand = experiment
    hand.read_latency = .11
    with pytest.raises(RuntimeError, match="Missed a full control period"):
        run(experiment)
    assert len(csv_rows(next(args.output_dir.iterdir()))) == 1
    assert hand.events[-2:] == ["disable", "disconnect"]


def test_missing_measurement_is_not_recorded_as_zero(experiment):
    args, _, _, hand = experiment
    hand.get_joint_position = lambda: SimpleNamespace(as_dict=lambda: {"index_mcp": None})
    with pytest.raises(RuntimeError, match="No finite position"):
        run(experiment)
    assert csv_rows(next(args.output_dir.iterdir())) == []
    assert hand.events[-2:] == ["disable", "disconnect"]


@pytest.mark.parametrize("failure", ["motor_ack", "disconnect_result"])
def test_non_exception_cleanup_failures_are_reported(experiment, failure):
    args, _, _, hand = experiment
    if failure == "motor_ack":
        hand.unacknowledged_motors = [3]
    else:
        hand.disconnect_result = (False, "torque disable failed while disconnecting")
    with pytest.raises(RuntimeError, match="cleanup failed"):
        run(experiment)
    assert hand.events[-2:] == ["disable", "disconnect"]
    metadata = json.loads(next(args.output_dir.glob("*/metadata.json")).read_text())
    assert metadata["status"] == "failed"


def test_installed_orca_core_mock_hand_api(tmp_path):
    """Exercise actual v2 joint/motor mapping and lifecycle without serial I/O."""
    core = pytest.importorskip("orca_core")
    config_path = tmp_path / "config.yaml"
    config_path.write_text(chirp.DEFAULT_CONFIG.read_text())
    args = chirp.make_parser().parse_args([
        "--config", str(config_path), "--duration", ".2", "--settle-time", "0",
        "--output-dir", str(tmp_path / "data"), "--joints", "thumb_cmc", "index_mcp",
    ])
    plan = chirp.make_plan(args, yaml.safe_load(config_path.read_text()))
    clock = Clock()
    hands = []

    def factory(**kwargs):
        hand = core.load_hand(**kwargs, mock=True)
        hands.append(hand)
        return hand

    run_dir = chirp.run_hardware(args, plan, factory=factory, clock=clock, sleep=clock.sleep)
    assert len(list(run_dir.glob("*.csv"))) == 2
    assert len(csv_rows(run_dir)) == 4
    assert all(np.isfinite(float(row["measured_position"])) for row in csv_rows(run_dir))
    assert not hands[0].is_connected()
    # Mock calibration must never be written over a real hand's calibration.
    assert not (tmp_path / "calibration.yaml").exists()


def test_existing_run_migration_preserves_data_and_is_idempotent(tmp_path):
    run_dir = tmp_path / "20261015_221742_718994"
    run_dir.mkdir()
    old_name = "chirp_wrist_20261015_221742_718994_ctrl20Hz_20.0s_0.1-5.0Hz_10deg.csv"
    data = b"timestamp,commanded_position,measured_position\r\n0.0,1.0,0.9\r\n"
    (run_dir / old_name).write_bytes(data)
    metadata_path = run_dir / "metadata.json"
    original = json.dumps(dict(joints=["wrist"], csv_files=[old_name], status="complete")).encode()
    metadata_path.write_bytes(original)
    rename_chirp.rename_results(run_dir)  # preview cannot change files
    assert metadata_path.read_bytes() == original
    assert (run_dir / old_name).read_bytes() == data
    rename_chirp.rename_results(run_dir, apply=True)
    metadata = json.loads(metadata_path.read_text())
    assert metadata["csv_files"] == ["wrist.csv"]
    assert metadata["started_at_utc"] == "2026-10-15T22:17:42.718994+00:00"
    assert (run_dir / "wrist.csv").read_bytes() == data
    assert (run_dir / ".metadata.before-rename.json").read_bytes() == original
    assert not (run_dir / old_name).exists()
    assert rename_chirp.rename_results(run_dir, apply=True) == []


def test_migration_preflights_all_collisions_before_renaming(tmp_path):
    names = ["chirp_wrist_old.csv", "chirp_thumb_cmc_old.csv"]
    for name in names:
        (tmp_path / name).write_text("original data")
    (tmp_path / "thumb_cmc.csv").write_text("existing data")
    original = json.dumps(dict(joints=["wrist", "thumb_cmc"], csv_files=names))
    (tmp_path / "metadata.json").write_text(original)
    with pytest.raises(FileExistsError):
        rename_chirp.rename_results(tmp_path, apply=True)
    assert (tmp_path / "metadata.json").read_text() == original
    assert (tmp_path / "thumb_cmc.csv").read_text() == "existing data"
    assert not (tmp_path / "wrist.csv").exists()
    assert all((tmp_path / name).exists() for name in names)
