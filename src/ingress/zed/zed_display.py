import pyzed.sl as sl
import cv2
import numpy as np


all_cameras = sl.Camera.get_device_list()
print("ALL AVAILABLE CAMERAS")
for cam in all_cameras:
    print(cam.serial_number, type(cam.serial_number))


# Create ZED camera object
zed = sl.Camera()

# Set configuration parameters
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720  # Choose from HD2K, HD1080, HD720, VGA
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Choose from ULTRA, QUALITY, PERFORMANCE, NONE

# Open the camera
status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(f"Camera failed to open: {status}")
    exit()

# Prepare image and depth containers
image = sl.Mat()
depth = sl.Mat()

runtime = sl.RuntimeParameters()

print("Press 'q' to quit.")
while True:
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image (color)
        zed.retrieve_image(image, sl.VIEW.LEFT)
        img_np = image.get_data()

        # Retrieve depth map
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        depth_np = depth.get_data()

        # Normalize depth for visualization
        depth_display = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = np.uint8(depth_display)

        # Display feeds
        cv2.imshow("ZED | Color", img_np)
        cv2.imshow("ZED | Depth", depth_display)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Cleanup
zed.close()
cv2.destroyAllWindows()
