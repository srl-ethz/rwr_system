import h5py
import numpy as np
import cv2
import os
from glob import glob
# Path to the HDF5 file and output video filename.

log_root = 'inference_logs'
video_out_root = "inference_vis"

# Open the HDF5 file
def extract_video(h5_file_path, video_output_path):
    with h5py.File(h5_file_path, 'r') as h5f:
        # Get all step keys (e.g., "step_0", "step_1", ...)
        step_keys = [key for key in h5f.keys() if key.startswith('step_')]
        # Sort keys by step number
        step_keys = sorted(step_keys, key=lambda x: int(x.split('_')[1]))
        frames = []
        for step in step_keys:
            front_key = f"{step}/observations/oakd_front_view_images"
            side_key = f"{step}/observations/oakd_wrist_bottom_view_images"
            wrist_key = f"{step}/observations/oakd_wrist_top_view_images"
            # Ensure that both keys exist for the step.
            if front_key in h5f and side_key in h5f:
                # Read images (shape: 3x244x244)
                front_img = h5f[front_key][()]  # e.g., returns a numpy array
                side_img = h5f[side_key][()]
                wrist_img = h5f[wrist_key][()]
                # Transpose to (244, 244, 3) for proper image format.
                front_img = np.transpose(front_img, (1, 2, 0))
                side_img = np.transpose(side_img, (1, 2, 0))
                wrist_img = np.transpose(wrist_img, (1, 2, 0))
                # Convert image values from [0,1] to [0,255] and cast to uint8.
                front_img = (front_img * 255).astype(np.uint8)[::-1,::-1]
                side_img = (side_img * 255).astype(np.uint8)[::-1,::-1]
                wrist_img = (wrist_img * 255).astype(np.uint8)
                # Concatenate the two images horizontally.
                concat_img = np.concatenate([front_img, side_img], axis=1)
                wrist_img = np.concatenate([np.zeros_like(wrist_img), wrist_img], axis=1)
                concat_img = np.concatenate([ wrist_img, concat_img], axis=0)
                frames.append(concat_img)
    # If we have extracted any frames, write them to an MP4 video.
    if frames:
        # Get dimensions from the first frame.
        height, width, _ = frames[0].shape
        # Set up the video writer.
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 10  # You can adjust the frame rate as needed.
        video_writer = cv2.VideoWriter(video_output_path, fourcc, fps, (width, height))
        for frame in frames:
            video_writer.write(frame)
        video_writer.release()
        print(f"Video saved as {video_output_path}")
    else:
        print("No frames were found in the HDF5 file.")

if __name__ == "__main__":
    for policy_dir in glob(os.path.join(log_root, "*")):
        print(policy_dir)
        os.makedirs(os.path.join(video_out_root, os.path.basename(policy_dir)), exist_ok=True)
        for log_file in glob(os.path.join(policy_dir,"*.hdf5" )):
            print(log_file)
            video_output_path = os.path.join(
                "inference_vis",
                os.path.basename(policy_dir),
                f"{os.path.basename(log_file).split('.')[0]}.mp4"
            )
            try:
                extract_video(log_file, video_output_path)
            except Exception as e:
                print(f"error generating video from {log_file}. Error message: \n {e}")