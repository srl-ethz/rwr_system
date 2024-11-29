import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R
class PointCloudVisualizer:
    def __init__(self, intrinsic_matrix, width, height, visualize=False):
        self.R_camera_to_world = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).astype(
            np.float64
        )
        self.depth_map = None
        self.rgb = None
        self.pcl = o3d.geometry.PointCloud()

        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width,
            height,
            intrinsic_matrix[0][0],
            intrinsic_matrix[1][1],
            intrinsic_matrix[0][2],
            intrinsic_matrix[1][2],
        )
        self.visualize = visualize
        if visualize:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name="Point Cloud")
            self.vis.add_geometry(self.pcl)
            origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.3, origin=[0, 0, 0]
            )
            self.vis.add_geometry(origin)
            view_control = self.vis.get_view_control()
            view_control.set_constant_z_far(1000)

        self.isstarted = False

    def rgbd_to_projection(self, depth_map, rgb, downsample=False, remove_noise=False):
        rgb_o3d = o3d.geometry.Image(rgb)
        depth_o3d = o3d.geometry.Image(depth_map)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d,
            depth_o3d,
            convert_rgb_to_intensity=(len(rgb.shape) != 3),
            depth_trunc=20000,
            depth_scale=1000.0,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, self.pinhole_camera_intrinsic
        )

        if downsample:
            pcd = pcd.voxel_down_sample(voxel_size=0.01)

        if remove_noise:
            pcd = pcd.remove_statistical_outlier(30, 0.1)[0]

        self.pcl.points = pcd.points
        self.pcl.colors = pcd.colors
        self.pcl.rotate(
            self.R_camera_to_world, center=np.array([0, 0, 0], dtype=np.float64)
        )
        return self.pcl, rgbd_image

    def visualize_pcd(self):
        if self.visualize:
            self.vis.update_geometry(self.pcl)
            self.vis.poll_events()
            self.vis.update_renderer()

    def close_window(self):
        if self.visualize:
            self.vis.destroy_window()


def calibrate_with_aruco(frame, camera_matrix, dist_coeffs, marker_length = 0.1):
    # Load camera calibration data (camera matrix and distortion coefficients)
    # You should replace these with the actual values from your camera calibration
    # Define the ArUco marker dictionary and the size of the marker (in meters)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    camera_matrix = np.array(camera_matrix)
    dist_coeffs = np.array(dist_coeffs)
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    # Detect ArUco markers in the image
    
    corners, ids, rejected = detector.detectMarkers(gray)

    # If at least one marker is detected
    if ids is not None:
        print(f"Detected {len(ids)} markers")
        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Loop through detected markers
        for i in range(len(ids)):
            print(f"Marker ID: {ids[i]}")
            print(f"Rotation Vector (rvec):\n {rvecs[i]}")
            print(f"Translation Vector (tvec):\n {tvecs[i]}\n")
    else:
        print("No markers found")
        return None

     
    
    # Convert the rotation vector (rvecs) to a 3x3 rotation matrix
    R_aruco_cam, _ = cv2.Rodrigues(rvecs[0])  # Convert rvec to a 3x3 rotation matrix

    # Initialize a 4x4 identity matrix for the transformation
    T_aruco_cam = np.eye(4)

    # Set the upper-left 3x3 part to the rotation matrix
    T_aruco_cam[:3, :3] = R_aruco_cam

    # Set the upper-right 3x1 part to the translation vector (tvec)
    T_aruco_cam[:3, 3] = tvecs[0].flatten()
    # Now T_cam_marker is the 4x4 transformation matrix from the camera to the marker
    T_cam_world = get_camera_in_world(T_aruco_cam)
    np.savetxt('T_cam_marker.txt', T_aruco_cam, fmt='%.6f')  # Save T_cam_marker with 6 decimal places
    np.savetxt('T_cam_world.txt', T_cam_world, fmt='%.6f')  # Save T_cam_world with 6 decimal places
    return T_cam_world


def get_camera_in_world(T_aruco_cam):
    """
    Calculate the transformation matrix of the camera in the world frame.
    
    Args:
        T_aruco_world (np.ndarray): 4x4 transformation matrix of ArUco in the world frame.
        T_aruco_cam (np.ndarray): 4x4 transformation matrix of ArUco in the camera frame.

    Returns:
        np.ndarray: 4x4 transformation matrix of the camera in the world frame.
    """
    T_aruco_world_position = np.array([0.694, -0.155, 0])
    T_aruco_world_euler = np.array([0, 0, 0 ])
    rotation_matrix = R.from_euler('xyz', T_aruco_world_euler).as_matrix()
    
    # Construct the 4x4 transformation matrix
    T_aruco_world = np.eye(4)
    T_aruco_world[:3, :3] = rotation_matrix
    T_aruco_world[:3, 3] = T_aruco_world_position
    
    # Compute the inverse of T_aruco_cam
    T_cam_aruco = np.linalg.inv(T_aruco_cam)
    
    # Compute T_cam_world
    T_cam_world = np.dot(T_aruco_world, T_cam_aruco)
    
    return T_cam_world

def compute_projection_matrix(K, T_cam_world):
    # Extract R and t from T_world_to_camera
    R = T_cam_world[:3, :3]
    t = T_cam_world[:3, 3]

    # Form the 3x4 extrinsics matrix [R | t]
    extrinsics = np.hstack((R, t.reshape(3, 1)))

    # Compute the projection matrix P
    P = K @ extrinsics
    return P