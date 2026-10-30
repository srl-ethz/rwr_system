#!/usr/bin/env python3
"""
Multi-OAK-D host driver with single main-loop and main-thread visualization.
"""

import argparse
import time
import threading
from threading import Thread, Lock
from dataclasses import dataclass
from collections import deque, defaultdict
from queue import Queue, Full, Empty
from copy import deepcopy
from typing import Optional, Tuple, Dict

import numpy as np
import depthai as dai
import cv2
import open3d as o3d

from oakd_utils import PointCloudVisualizer


# ---------- Stereo / camera config ----------
COLOR = True           # Color RGB stream; when False, "colorize" = rectified right # Chenyu: GPT wrote this, TODO check if it is necessary.
LRCHECK = True         # Better handling for occlusions
EXTENDED = True        # Closer min depth (double disparity range)
SUBPIXEL = True        # Higher accuracy at long range
MEDIAN = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7

# ---------- Synchronizer for streams within one device ----------
class HostSync:
    """Synchronize multiple DepthAI streams by sequence number."""
    def __init__(self):
        self.buffers: Dict[str, list] = {}

    def add_msg(self, name: str, msg):
        if name not in self.buffers:
            self.buffers[name] = []
        self.buffers[name].append({"msg": msg, "seq": msg.getSequenceNum()})

        synced = {}
        for key, arr in self.buffers.items():
            for obj in arr:
                if obj["seq"] == msg.getSequenceNum():
                    synced[key] = obj["msg"]
                    break

        if self.buffers.keys() == synced.keys():
            # Trim old frames
            for key, arr in self.buffers.items():
                self.buffers[key] = [o for o in arr if o["seq"] >= msg.getSequenceNum()]
            return synced
        return False


# ---------- Pipeline factory ----------
def create_pipeline(use_depth: bool, use_color: bool = True) -> dai.Pipeline:
    print("StereoDepth config options:",
        f"\n    Left-Right check:   {LRCHECK}",
        f"\n    Extended disparity: {EXTENDED}",
        f"\n    Subpixel:           {SUBPIXEL}",
        f"\n    Median filtering:   {MEDIAN}")

    pipeline = dai.Pipeline()

    # XLink outs
    xout_colorize = pipeline.createXLinkOut()
    xout_colorize.setStreamName("colorize")

    if use_color:
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setIspScale(1, 2)  # 1920x1080 -> 960x540
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        camRgb.initialControl.setManualFocus(130)
        camRgb.isp.link(xout_colorize.input)

    if use_depth:
        # Mono cameras
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Stereo
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo.initialConfig.setMedianFilter(MEDIAN)
        stereo.initialConfig.setConfidenceThreshold(180)
        stereo.setLeftRightCheck(LRCHECK)
        stereo.setExtendedDisparity(EXTENDED)
        stereo.setSubpixel(SUBPIXEL)

        # Post-processing
        cfg = stereo.initialConfig.get()
        cfg.postProcessing.speckleFilter.enable = False
        cfg.postProcessing.speckleFilter.speckleRange = 50
        cfg.postProcessing.temporalFilter.enable = False
        cfg.postProcessing.spatialFilter.enable = True
        cfg.postProcessing.spatialFilter.holeFillingRadius = 5
        cfg.postProcessing.spatialFilter.numIterations = 2
        cfg.postProcessing.thresholdFilter.minRange = 200
        cfg.postProcessing.thresholdFilter.maxRange = 1000
        cfg.postProcessing.decimationFilter.decimationFactor = 1
        stereo.initialConfig.set(cfg)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        if use_color:
            stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        else:
            # If no color cam, just pass rectified right to "colorize"
            stereo.rectifiedRight.link(xout_colorize.input)

        # Outputs
        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        xout_rect_left = pipeline.createXLinkOut()
        xout_rect_right = pipeline.createXLinkOut()
        xout_rect_left.setStreamName("rectified_left")
        xout_rect_right.setStreamName("rectified_right")
        stereo.rectifiedLeft.link(xout_rect_left.input)
        stereo.rectifiedRight.link(xout_rect_right.input)

    return pipeline


# ---------- Driver: producer thread only ----------
class OakDDriver:
    """
    Reads from one physical OAK device, synchronizes streams, and pushes FramePackets to a queue.
    Never performs visualization or blocking GUI ops.
    """
    def __init__(self, callback, device_mxid: Optional[str], camera_name: str, use_depth: bool, ir_brightness: int = 1200):
        self.callback = callback
        self.device_mxid = device_mxid
        self.camera_name = camera_name
        self.use_depth = use_depth
        self.ir_brightness = ir_brightness

        self._stop = threading.Event()
        self.thread = Thread(target=self._run, daemon=True)
        self.thread.start()
        self.intrinsics = None
        self.distortion_coeff = None
        self.check_usb_speed_every = 300


    def stop(self):
        self._stop.set()
        self.thread.join(timeout=2.0)


    def _run(self):
        pipeline = create_pipeline(use_depth=self.use_depth, use_color=COLOR)

        # Open device
        if self.device_mxid is None:
            device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER)  # first available
            print(f"[{self.camera_name}] Using first available device: {device.getDeviceInfo()}")
        else:
            info = dai.DeviceInfo(self.device_mxid)
            device = dai.Device(pipeline, info, maxUsbSpeed=dai.UsbSpeed.SUPER)
            print(f"[{self.camera_name}] Using device MXID: {self.device_mxid}")

        with device:
            serial_no = device.getMxId()
            try:
                device.setIrLaserDotProjectorIntensity(self.ir_brightness)
            except Exception as e:
                print(f"[{self.camera_name}] Warning: couldn't set IR projector: {e}")


            # Prepare output queues
            qnames = ["colorize"]
            if self.use_depth:
                qnames += ["depth"] #, "rectified_left", "rectified_right"]

            queues = {name: device.getOutputQueue(name, maxSize=2, blocking=False) for name in qnames}
            sync = HostSync()

            # Calibration (fill intrinsics/distortion once we know frame size)
            calib = device.readCalibration()
            has_intr = False
            check_usb_speed_cnt = self.check_usb_speed_every

            while not self._stop.is_set():
                check_usb_speed_cnt -= 1
                if check_usb_speed_cnt<=0:
                    if device.getUsbSpeed() != dai.UsbSpeed.SUPER:
                        print(f"[{self.camera_name}] WARNING: USB Speed {device.getUsbSpeed()}")
                    check_usb_speed_cnt = self.check_usb_speed_every

                got_any = False
                for name, q in queues.items():
                    msg = q.tryGet()
                    if msg is None:
                        continue
                    got_any = True
                    synced = sync.add_msg(name, msg)
                    if not synced:
                        continue

                    # Pull frames
                    color = synced["colorize"].getCvFrame()
                    stamp = synced["colorize"].getTimestamp()
                    seq = synced["colorize"].getSequenceNum()

                    depth = None
                    if self.use_depth:
                        try:
                            depth = synced["depth"].getFrame()
                            # rect_l = synced["rectified_left"].getCvFrame()
                            # rect_r = synced["rectified_right"].getCvFrame()
                        except Exception:
                            # In case any stream is momentarily missing
                            pass

                    # Resolve intrinsics for the current "colorize" size
                    if not has_intr:
                        h, w = color.shape[:2]
                        if COLOR:
                            self.intrinsics = np.array(
                                calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, dai.Size2f(w, h))
                            )
                            self.distortion_coeff = np.array(
                                calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
                            )
                        else:
                            # "colorize" == rectified right
                            self.intrinsics = np.array(
                                calib.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, dai.Size2f(w, h))
                            )
                            self.distortion_coeff = np.array(
                                calib.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT)
                            )
                        has_intr = True

                    if self.callback is not None:
                        if self.camera_name is not None:
                            self.callback(color, depth, stamp, self.camera_name)
                        else:
                            self.callback(color, depth, stamp)

                if not got_any:
                    time.sleep(0.001)  # yield a bit to avoid spinning


# ---------- Helper: depth visualization ----------
def colorize_depth_mm(depth_mm: np.ndarray) -> np.ndarray:
    """Convert uint16 depth (mm) to a HOT color map for display."""
    if depth_mm is None:
        return None
    d8 = cv2.normalize(depth_mm, None, 255, 0, cv2.NORM_INF, cv2.CV_8U)
    d8 = cv2.equalizeHist(d8)
    return cv2.applyColorMap(d8, cv2.COLORMAP_HOT)

# ---------- Packets passed from driver threads to main loop ----------
@dataclass
class FramePacket:
    camera_name: str
    color: np.ndarray                     # BGR frame (H, W, 3)
    stamp: any                            # DepthAI timestamp of "colorize" frame

    depth: Optional[np.ndarray] = None    # uint16 in millimeters

# ---------- Main ----------
def main():
    parser = argparse.ArgumentParser(description="Multi-OAK-D streamer with main-thread visualization.")
    parser.add_argument("--visualize", dest="visualize", action="store_true", help="Enable visualization windows.")
    parser.add_argument("--no-visualize", dest="visualize", action="store_false", help="Disable visualization (default).")
    parser.set_defaults(visualize=False)

    parser.add_argument("--depth", dest="use_depth", action="store_true", help="Enable depth/pointcloud (default).")
    parser.add_argument("--no-depth", dest="use_depth", action="store_false", help="Disable depth/pointcloud.")
    parser.set_defaults(use_depth=True)

    parser.add_argument("--ir", type=int, default=1200, help="IR laser dot projector brightness (default: 1200).")
    args = parser.parse_args()

    # Detect devices
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("No OAK-D devices found.")
        raise SystemExit(1)

    print("\n=== Connected OAK-D devices ===")
    for i, dev in enumerate(devices):
        try:
            mxid = dev.getMxId()
            state = getattr(dev, "state", "Unknown")
            print(f"[{i}] MXID: {mxid} | State: {state}")
        except Exception:
            print(f"[{i}] {dev}")
    print("================================\n")


    queue: Queue[FramePacket] = Queue(maxsize=30)
    def driver_callback(color, depth, stamp, camera_name=None):
        pkt = FramePacket(
            camera_name=camera_name,
            color=color,
            stamp=stamp,
            depth=depth
        )

        # Non-blocking push with overflow drop of oldest
        try:
            queue.put_nowait(pkt)
        except Full:
            try:
                _ = queue.get_nowait()
            except Empty:
                pass
            queue.put_nowait(pkt)

    # Start drivers (producers)
    drivers: list[OakDDriver] = []
    for idx, dev in enumerate(devices):
        mxid = dev.getMxId()
        cam_name = f"OAKD-{idx}-{mxid}"  # short suffix for readability
        d = OakDDriver(driver_callback, device_mxid=mxid, camera_name=cam_name, use_depth=args.use_depth, ir_brightness=args.ir)
        drivers.append(d)

    # Build main-thread main_worker (consumes packets, does visualization)
    trackers = {}  # cam_name -> dict(ts deque, lat deque, lock)

    def _ensure_tracker(cam):
        if cam not in trackers:
            trackers[cam] = {"ts": deque(maxlen=120), "lat": deque(maxlen=120), "lock": Lock()}

    def _update_stats(cam: str, pkt: FramePacket):
        _ensure_tracker(cam)
        host_now = dai.Clock.now()
        # Convert latency (host_now - device_stamp) to milliseconds (guard negatives)
        dt = (host_now - pkt.stamp)
        try:
            latency_ms = max(0.0, dt.total_seconds() * 1000.0)
        except Exception:
            # Fallback
            latency_ms = 0.0
        tr = trackers[cam]
        with tr["lock"]:
            tr["ts"].append(pkt.stamp)
            tr["lat"].append(latency_ms)

    last_stats_print = {"t": 0.0}

    def _print_stats_once_per_sec():
        now = time.time()
        if now - last_stats_print["t"] < 1.0:
            return
        last_stats_print["t"] = now
        parts = []
        for cam, tr in trackers.items():
            with tr["lock"]:
                dq_ts = tr["ts"]; dq_lat = tr["lat"]
                # FPS
                if len(dq_ts) >= 2:
                    dt = (dq_ts[-1] - dq_ts[0]).total_seconds()
                    fps = (len(dq_ts) - 1) / dt if dt > 1e-6 else 0.0
                else:
                    fps = 0.0
                # Avg latency (ms)
                avg_lat = (sum(dq_lat) / len(dq_lat)) if dq_lat else 0.0
            parts.append(f"{cam}: {fps:5.2f} FPS, {avg_lat:6.2f} ms")
        if parts:
            print("[STATS]", " | ".join(parts))

    def main_worker(pkt: FramePacket):
        cam = pkt.camera_name

        # Track stats
        _update_stats(cam, pkt)
        _print_stats_once_per_sec()

        if args.visualize:
            # Show color and depth
            cv2.imshow(f"{cam}_color", pkt.color)
            if args.use_depth and pkt.depth is not None:
                cv2.imshow(f"{cam}_depth", colorize_depth_mm(pkt.depth))

    print("Streaming… ('q' to quit).")
    try:
        while True:
            got_any = False
            for d in drivers:
                try:
                    pkt = queue.get_nowait()
                except Empty:
                    continue
                got_any = True
                # IMPORTANT: visualization happens here (main thread)
                main_worker(pkt)

            if args.visualize:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

            if not got_any:
                time.sleep(0.001)  # idle politely
    except KeyboardInterrupt:
        pass
    finally:
        for d in drivers:
            d.stop()
        if args.visualize:
            cv2.destroyAllWindows()
        print("Stopped.")

if __name__ == "__main__":
    main()



