import h5py
import cv2
import numpy as np
import shutil
import time
import os
import argparse

def process_h5_file_gray_with_color(input_file, color, output_file=None):
    # Validate color input
    color = color.lower()
    color_channels = {'blue': 2, 'yellow': 1, 'red': 0}
    if color not in color_channels:
        raise ValueError("Color must be one of 'blue', 'yellow', or 'red'.")

    channel_idx = color_channels[color]

    # Determine output file path
    if output_file is None:
        # Create a temporary file to avoid corrupting the original in case of errors
        temp_output = input_file.split(".")[0]+ "_grayed.h5"
        print(temp_output)
        shutil.copyfile(input_file, temp_output)
        output_file = temp_output
    else:
        shutil.copyfile(input_file, output_file)
        
        
    assert os.path.exists(output_file), f"Output file {output_file} does not exist."
    # Open the HDF5 file in read/write mode
    with h5py.File(output_file, 'r+') as hf:
        # Define the views to process
        views = ['oakd_front_view', 'oakd_side_view', 'oakd_wrist_view']
        
        for view in views:
            # Construct dataset paths
            color_path = f'observations/images/{view}/color'
            mask_path = f'observations/images/{view}/color_mask'

            # Check if datasets exist
            if color_path not in hf or mask_path not in hf:
                print(f"Skipping {view}: Missing 'color' or 'color_mask' dataset.")
                continue

            # Access the datasets
            color_dataset = hf[color_path]
            mask_dataset = hf[mask_path]

            num_images = len(color_dataset)

            for i in range(num_images):
            
                # Read the i-th image and mask
                color_image = color_dataset[i]  # Shape: (H, W, 3) BGR
                color_mask = mask_dataset[i]    # Shape: (H, W, 3)
                
                # Verify dimensions
                if color_image.shape[:2] != color_mask.shape[:2]:
                    print(f"Skipping image {i} in {view}: Image and mask dimensions do not match.")
                    continue

                # Split the color mask into channels
                if color_mask.ndim == 3 and color_mask.shape[2] >= 3:
                    mask_channels = cv2.split(color_mask)
                    mask_channel = mask_channels[channel_idx]

                # Convert the color image to grayscale
                rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                
                # gray_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
                gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                gray_image_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

                condition_mask = (mask_channel == 255)
                
                modified_image = np.copy(gray_image_bgr)
                modified_image[condition_mask,:] = (255,0,0)
                
                color_dataset[i] = modified_image
                
                cv2.imwrite('colored_grayscale_images/gray_blue.png', modified_image)
                cv2.imwrite('colored_grayscale_images/original.png', rgb)

    # If output_file is a temporary file and processing was successful, replace the original file

    print("Processing completed successfully, saved to:", output_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process HDF5 files to grayscale with color highlights.')
    parser.add_argument('input_folder', type=str, help='Path to the input folder containing HDF5 files.')
    parser.add_argument('--color', type=str, default='blue', help='Color to highlight (blue, yellow, or red).')
    parser.add_argument('--output_folder', type=str, help='Path to the output folder. Default is at the same level as input folder with suffix "_grayed".')

    args = parser.parse_args()

    input_folder = args.input_folder
    color = args.color
    output_folder = args.output_folder

    if output_folder is None:
        output_folder = os.path.join(os.path.dirname(input_folder), os.path.basename(input_folder) + '_grayed')

    os.makedirs(output_folder, exist_ok=True)

    for file in os.listdir(input_folder):
        if file.endswith('.h5'):
            input_file = os.path.join(input_folder, file)
            
            input_folder_clean = os.path.normpath(input_folder) 
            parent_dir = os.path.dirname(input_folder_clean)
            base_name = os.path.basename(input_folder_clean)
            output_folder_name = f"{base_name}_grayed"
                    
            output_folder = os.path.join(parent_dir, output_folder_name)
            print("Output folder:", output_folder)
            os.makedirs(output_folder, exist_ok=True)
            
            output_file = os.path.join(output_folder, file)
            process_h5_file_gray_with_color(input_file, color, output_file)
            print("Processed", file)
