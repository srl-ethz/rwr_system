"""Local ROS integration tests. Run with a separate domain; no robot drivers start."""
from pathlib import Path
import signal
import subprocess
import sys
import time
import json
import tempfile

import h5py
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from ingress.msg import CameraIntrinsics
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
import yaml

from faive_system.src.logger.logger_node import DemoLogger
from faive_system.src.logger.syncronize_data import sample_and_sync_h5
from faive_system.experiments.scripts.inspect_demonstration import inspect

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "experiments/cfgs/logger/datacollection_right.yaml"


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=8)
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()
        pytest.fail(f"Process did not stop after SIGINT:\n{output}")
    return output


def test_right_only_service_mode_starts_without_robot():
    process = subprocess.Popen(
        [sys.executable, "-u", str(ROOT / "experiments/run_teleop_rokoko.py"),
         "--ros-args", "-p", "enable_left:=false", "-p", "enable_right:=true", "-p", "interactive:=false"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        time.sleep(2)
        running = process.poll() is None
    finally:
        output = stop_process(process)
    assert running, output
    assert "waiting for pause/resume service calls" in output
    assert "AttributeError" not in output
    assert "Traceback" not in output
    assert process.returncode == 0


def test_rokoko_without_udp_stream_stops_cleanly():
    process = subprocess.Popen(
        [sys.executable, "-u", str(ROOT / "src/ingress/rokoko/rokoko_node.py"),
         "--ros-args", "-p", "rokoko_tracker/port:=0"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        time.sleep(2)
        running = process.poll() is None
    finally:
        output = stop_process(process)
    assert running, output
    assert "waiting for hand tracker" not in output
    assert process.returncode == 0, output


def test_hand_only_launch_does_not_start_arm_or_hardware_controller():
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen(
        ['ros2', 'launch', 'experiments', 'teleop_rokoko.launch.py',
         'teleop_arm:=false', 'enable_hand_controller:=false'],
        stdout=log, stderr=subprocess.STDOUT, text=True,
    )
    rclpy.init()
    observer = Node('hand_only_launch_observer')
    try:
        deadline = time.monotonic() + 15
        services = []
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            services = [name for name, _ in observer.get_service_names_and_types()]
            if '/right/retargeter/set_enabled' in services:
                break
            assert process.poll() is None, 'Hand-only launch exited during startup'
        assert '/right/retargeter/set_enabled' in services
        assert not any(name.startswith('/franka_teleop/') for name in services)
        assert '/right/hand/estop' not in services
        assert 'keyboard_runner_node' not in observer.get_node_names()
    finally:
        observer.destroy_node()
        rclpy.try_shutdown()
        stop_process(process)
        log.seek(0)
        output = log.read()
        log.close()
    assert 'Traceback' not in output, output
    assert process.returncode == 0, output


def test_best_effort_images_survive_record_convert_sync_and_preview(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"topics_and_types_path:={CONFIG}",
                     "-p", f"storage_base_path:={tmp_path}", "-p", "data_name:=synthetic"])
    logger = DemoLogger()
    publisher = Node("synthetic_demonstration")
    executor = SingleThreadedExecutor()
    executor.add_node(logger)
    executor.add_node(publisher)
    config = yaml.safe_load(CONFIG.read_text())
    pubs = {}
    for topic, (_, cls) in logger.topics_and_types.items():
        qos = QoSProfile(depth=10)
        if cls is Image:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        elif cls is CameraIntrinsics:
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pubs[topic] = publisher.create_publisher(cls, topic, qos)
    try:
        assert logger.start_recording_cb(Trigger.Request(), Trigger.Response()).success
        deadline = time.monotonic() + 8
        while not all(pub.get_subscription_count() for pub in pubs.values()):
            assert time.monotonic() < deadline, "Logger did not match all publishers (including image QoS)"
            executor.spin_once(timeout_sec=0.02)
        for i in range(30):
            for topic, pub in pubs.items():
                cls = logger.topics_and_types[topic][1]
                message = cls()
                if cls is Image:
                    message.height, message.width = 24, 32
                    message.encoding, message.step = "rgb8", 32 * 3
                    message.data = np.full((24, 32, 3), i * 5, dtype=np.uint8).tobytes()
                elif cls is PoseStamped:
                    message.pose.position.x = 0.4 + i * 0.001
                    message.pose.orientation.w = 1.0 if i % 2 == 0 else -1.0
                elif cls is CameraIntrinsics:
                    message.fx = message.fy = 100.0
                    message.cx, message.cy = 16.0, 12.0
                    message.disto = [0.0] * 5
                else:
                    message.data = [i * 0.001] * 17
                pub.publish(message)
            deadline = time.monotonic() + 0.04
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.005)
        assert logger.stop_recording_cb(Trigger.Request(), Trigger.Response()).success
        assert logger.save_recording_cb(Trigger.Request(), Trigger.Response()).success
    finally:
        executor.shutdown()
        publisher.destroy_node()
        logger.destroy_node()
        rclpy.try_shutdown()
    raw = next((tmp_path / "synthetic").glob("*.h5"))
    assert inspect(raw, ["oakd_front_view", "oakd_wrist_view"])["ok"]
    processed = tmp_path / "processed.h5"
    sample_and_sync_h5(config["keys_and_topics"], config["topics_and_types"], raw, processed, 10, True, None)
    with h5py.File(processed, "r") as handle:
        assert handle["observations/right/qpos_hand"].shape[1] == 17
        assert handle["observations/images/oakd_front_view/intrinsics"].shape == (3, 3)
        assert np.isfinite(handle["observations/right/qpos_arm"][:]).all()
    preview = tmp_path / "preview"
    assert inspect(processed, ["oakd_front_view", "oakd_wrist_view"], preview)["ok"]
    assert (preview / "preview.mp4").stat().st_size > 100
    assert (preview / "trajectory.png").exists()
    with h5py.File(processed, "a") as handle:
        del handle["observations/images/oakd_wrist_view/color"]
    assert not inspect(processed, ["oakd_front_view", "oakd_wrist_view"])["ok"]


def test_synchronization_rejects_disjoint_streams(tmp_path):
    raw = tmp_path / "raw.h5"
    with h5py.File(raw, "w") as handle:
        for topic, stamps in [("/a", [1_000_000_000, 2_000_000_000]), ("/b", [3_000_000_000, 4_000_000_000])]:
            group = handle.create_group(topic)
            for stamp in stamps:
                group.create_dataset(str(stamp), data=[0.0] * 17)
    keys = {"{0}": {"dtype": "std_msgs.msg.Float32MultiArray", "topic": "/(.*)"}}
    types = {name: "std_msgs.msg.Float32MultiArray" for name in ["/a", "/b"]}
    with pytest.raises(ValueError, match="no shared time interval"):
        sample_and_sync_h5(keys, types, raw, tmp_path / "bad.h5", 10, False, None)


@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_v2_retargeter_outputs_hardware_joint_order(device):
    import xacro
    import torch
    from faive_system.src.retargeter.retargeter import Retargeter
    import orca_core
    if device == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")

    model = ROOT / "src/viz/models/orcahand_v2"
    cfg = ROOT / "experiments/cfgs/retargeter"
    scheme = yaml.safe_load((model / "scheme_orcahand_v2.yaml").read_text())
    hardware = yaml.safe_load((ROOT / "src/hand_control/orca/configs/orcahand_v2/right/config.yaml").read_text())
    assert list(scheme["gc_tendons"]) == hardware["joint_ids"]
    urdf = xacro.process_file(str(model / "orcahand_v2.urdf.xacro"), mappings={"hand_chirality": "right"}).toxml()
    retargeter = Retargeter(
        urdf_string=urdf, hand_scheme=scheme, device=device,
        mano_adjustments=str(cfg / "mano_adjustment_orcahand_v2.yaml"),
        retargeter_cfg=str(cfg / "retargeter_cfgs_orcahand_v2.yaml"),
    )
    poses = json.loads((ROOT / "tests/artifacts/handposes_20241127222652.json").read_text())
    for entry in poses[:3]:
        pose = entry[1] if len(entry) == 3 else entry[0]
        pose = np.asarray(pose)
        if pose.shape == (21, 3):
            # The historical fixture lacks the forearm. Add a synthetic forearm
            # for this shape/order smoke test; real ingress supplies all 22 points.
            pose = np.vstack([2 * pose[0] - pose[9], pose])
        angles, _ = retargeter.retarget(pose, {})
        assert np.asarray(angles).shape == (17,)
        assert np.isfinite(angles).all()
