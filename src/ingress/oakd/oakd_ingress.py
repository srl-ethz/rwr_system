#!/usr/bin/env python3

# Code copied from main depthai repo, depthai_helpers/projector_3d.py

import depthai as dai
import cv2
import time
import open3d as o3d
import threading
from oakd_utils import PointCloudVisualizer, calibrate_with_aruco, compute_projection_matrix
import numpy as np
import yaml
import os
#!/usr/bin/env python3

COLOR = True

lrcheck = True  # Better handling for occlusions
extended = False  # Closer-in minimum depth, disparity range is doubled
subpixel = True  # Better accuracy for longer distance, fractional disparity 32-levels
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7
median = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7

print("StereoDepth config options:")
print("    Left-Right check:  ", lrcheck)
print("    Extended disparity:", extended)
print("    Subpixel:          ", subpixel)
print("    Median filtering:  ", median)

pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.MonoCamera)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

monoRight = pipeline.create(dai.node.MonoCamera)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo = pipeline.createStereoDepth()
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.initialConfig.setMedianFilter(median)
# stereo.initialConfig.setConfidenceThreshold(255)

stereo.setLeftRightCheck(lrcheck)
stereo.setExtendedDisparity(extended)
stereo.setSubpixel(subpixel)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = False
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = False
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 1
config.postProcessing.thresholdFilter.minRange = 400
config.postProcessing.thresholdFilter.maxRange = 200000
config.postProcessing.decimationFilter.decimationFactor = 1
stereo.initialConfig.set(config)

xout_depth = pipeline.createXLinkOut()
xout_depth.setStreamName("depth")
stereo.depth.link(xout_depth.input)

# xout_disparity = pipeline.createXLinkOut()
# xout_disparity.setStreamName('disparity')
# stereo.disparity.link(xout_disparity.input)

xout_colorize = pipeline.createXLinkOut()
xout_colorize.setStreamName("colorize")
xout_rect_left = pipeline.createXLinkOut()
xout_rect_left.setStreamName("rectified_left")
xout_rect_right = pipeline.createXLinkOut()
xout_rect_right.setStreamName("rectified_right")

if COLOR:
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setIspScale(1, 2)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.initialControl.setManualFocus(130)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    camRgb.isp.link(xout_colorize.input)
else:
    stereo.rectifiedRight.link(xout_colorize.input)

stereo.rectifiedLeft.link(xout_rect_left.input)
stereo.rectifiedRight.link(xout_rect_right.input)


class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []
        # Add msg to array
        self.arrays[name].append({"msg": msg, "seq": msg.getSequenceNum()})

        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                if msg.getSequenceNum() == obj["seq"]:
                    synced[name] = obj["msg"]
                    break
        if self.arrays.keys() == synced.keys():
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if obj["seq"] < msg.getSequenceNum():
                        arr.remove(obj)
                    else:
                        break
            return synced
        else:
            return False

# Detect cameras and asign them according to oakd_cams.yaml
available_cams = dai.Device.getAllAvailableDevices()
print(f"Available devices: \n")
for c in available_cams:
    print(c)

oakd_cams_file = os.path.join(os.path.dirname(__file__), "oakd_cams.yaml")
with open(oakd_cams_file, "r") as cfgf:
    oakd_cams = yaml.load(cfgf, Loader=yaml.FullLoader)

OAK_CAMS_LIST = {} #from name to id

for c in available_cams:
    # print(type(c.mxid), c.mxid)
    if c.mxid in oakd_cams.keys():
        OAK_CAMS_LIST[oakd_cams[c.mxid]] = c.mxid
    else:
        print(f"WARNING! Camera {c.mxid} detected but not configed in ${oakd_cams_file}")

for mxid in set(oakd_cams.keys()).difference([c.mxid for c in available_cams]):
    print(f"WARNING! Camera {mxid} is listed in config file but not currently detected")

print("OAK_CAMS_LIST", OAK_CAMS_LIST)

class OakDDriver:

    def __init__(
        self, callback, visualize=True, device_mxid=None, camera_name=None
    ) -> None:
        print(f"Using OAK-D device {device_mxid}")
        print(f"Using camera: {camera_name}")

        self.visualize = visualize
        self.callback = callback
        self.device_mxid = device_mxid
        self.camera_name = camera_name
        self.thread = threading.Thread(target=self.run_thread)
        self.thread.daemon = True
        self.thread.start()
        self.intrinsics = None
        self.inv_intrinsics = None
        self.distortion_coeff = None
        self.calibrated = False
    
    def calibrate(self, frame):
        T_camera_marker = calibrate_with_aruco(frame, self.intrinsics, self.distortion_coeff)
        if T_camera_marker is None:
            print("Calibration failed")
            return
        self.extrinsics = T_camera_marker
        self.projection_matrix = compute_projection_matrix(self.intrinsics, T_camera_marker)
        self.calibrated = True

    def print_devices(self):
        for device in dai.Device.getAllAvailableDevices():
            print(f"{device.getMxId()} {device.state}")

    def run_thread(self):
        if self.device_mxid is None:
            device = dai.Device(pipeline)
            print("No device specified, using first available device")
            print("Devide Info: ", device.getDeviceInfo())
        else:
            device_info = dai.DeviceInfo(self.device_mxid)
            device = dai.Device(pipeline, device_info)

        self.run(device)

    def run(self, device):
        with device:
            print("Starting pipeline...")
            attempts = 1000
            has_depth = False
            for _ in range(attempts):
                print("Trying to get depth stream")
                if device.getOutputQueue("depth", maxSize=1, blocking=False).tryGet() is not None:
                    has_depth = True
                    print("OAK-D detected")
                    break
                time.sleep(0.1)

                
            device.setIrLaserDotProjectorBrightness(1200)
            qs = []
            qs.append(device.getOutputQueue("colorize", maxSize=1, blocking=False))
            if has_depth:
                qs.append(device.getOutputQueue("depth", maxSize=1, blocking=False))
                qs.append(
                    device.getOutputQueue("rectified_left", maxSize=1, blocking=False)
                )
                qs.append(
                    device.getOutputQueue("rectified_right", maxSize=1, blocking=False)
                )

            calibData = device.readCalibration()
            if COLOR:
                w, h = camRgb.getIspSize()
                intrinsics = calibData.getCameraIntrinsics(
                    dai.CameraBoardSocket.RGB, dai.Size2f(w, h)
                )

                distortion_coeff = np.array(
                    calibData.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
                )

            else:
                w, h = monoRight.getResolutionSize()
                intrinsics = calibData.getCameraIntrinsics(
                    dai.CameraBoardSocket.RIGHT, dai.Size2f(w, h)
                )
                distortion_coeff = np.array(
                    calibData.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT)
                )

            self.intrinsics = intrinsics
            self.inv_intrinsics = np.linalg.inv(intrinsics)
            self.distortion_coeff = distortion_coeff
            
            
            if has_depth:
                pcl_converter = PointCloudVisualizer(intrinsics, w, h, self.visualize)

            serial_no = device.getMxId()
            sync = HostSync()
            depth_vis, color, rect_left, rect_right = None, None, None, None

            while True:
                for q in qs:
                    new_msg = q.tryGet()
                    if new_msg is not None:
                        msgs = sync.add_msg(q.getName(), new_msg)
                        if msgs:
                            if has_depth:
                                try:
                                    depth = msgs["depth"].getFrame()
                                    color = msgs["colorize"].getCvFrame()
                                    rectified_left = msgs["rectified_left"].getCvFrame()
                                    rectified_right = msgs["rectified_right"].getCvFrame()
                                except Exception as e:
                                    print(e)
                                    continue
                            color = msgs["colorize"].getCvFrame()
                            if self.calibrated == False:
                                self.calibrate(color)

                            if self.visualize:
                                cv2.imshow("color", color)
                                if has_depth:
                                    depth_vis = cv2.normalize(
                                        depth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1
                                    )
                                    depth_vis = cv2.equalizeHist(depth_vis)
                                    depth_vis = cv2.applyColorMap(
                                        depth_vis, cv2.COLORMAP_HOT
                                    )
                                    cv2.imshow("depth", depth_vis)
                                    cv2.imshow("rectified_left", rectified_left)
                                    cv2.imshow("rectified_right", rectified_right)

                            rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                            if has_depth and self.visualize and depth is not None:
                                pcd, rgbd = pcl_converter.rgbd_to_projection(depth, rgb)

                            if self.visualize and has_depth:
                                pcl_converter.visualize_pcd()

                            if self.callback is not None:
                                if self.camera_name is not None:
                                    self.callback(color, depth, self.camera_name)
                                else:
                                    self.callback(color, depth)

                key = cv2.waitKey(1)
                if key == ord("s"):
                    timestamp = str(int(time.time()))
                    cv2.imwrite(f"{serial_no}_{timestamp}_color.png", color)
                    if has_depth:
                        cv2.imwrite(f"{serial_no}_{timestamp}_depth.png", depth_vis)
                        cv2.imwrite(
                            f"{serial_no}_{timestamp}_rectified_left.png", rectified_left
                        )
                        cv2.imwrite(
                            f"{serial_no}_{timestamp}_rectified_right.png", rectified_right
                        )
                        o3d.io.write_point_cloud(
                            f"{serial_no}_{timestamp}.pcd",
                            pcl_converter.pcl,
                            compressed=True,
                        )
                elif key == ord("q"):
                    break


if __name__ == "__main__":
    OakDDriver(None, visualize=True, device_mxid=None)
    while True:
        time.sleep(1)
