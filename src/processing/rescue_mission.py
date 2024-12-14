import h5py
import os
import numpy as np
import sys
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def calculate_averages(directory, required_keys):
    """Calculate the average values for the required keys from all HDF5 files in the directory."""
    averages = {key: [] for key in required_keys}
    file_count = 0

    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        if file_path.endswith('.h5') and os.path.isfile(file_path):
            with h5py.File(file_path, 'r') as f:
                for key in required_keys:
                    if key in f:
                        averages[key].append(f[key][:])
            file_count += 1

    for key in averages:
        if averages[key]:
            averages[key] = np.mean(averages[key], axis=0)
        else:
            averages[key] = None

    return averages, file_count

def add_missing_keys(file_path, required_keys, averages, file_count):
    """Add missing keys to an HDF5 file with average values if available."""
    try:
        with h5py.File(file_path, 'a') as f:
            missing_keys = []
            for key in required_keys:
                if key not in f:
                    if averages[key] is not None and file_count > 1:
                        f.create_dataset(key, data=averages[key])
                        logging.info(f"Added missing key {key} to {file_path} with average value.")
                    else:
                        missing_keys.append(key)
            
            if missing_keys:
                logging.info(f"Missing keys in {file_path} not added due to lack of average values: {missing_keys}")
    except Exception as e:
        logging.error(f"Error reading {file_path}: {e}")

def process_directory(directory, required_keys):
    """Process all .h5 files in a directory."""
    averages, file_count = calculate_averages(directory, required_keys)
    
    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        if file_path.endswith('.h5') and os.path.isfile(file_path):
            add_missing_keys(file_path, required_keys, averages, file_count)

if __name__ == "__main__":
    # Check if directory path is provided as a command line argument
    if len(sys.argv) != 2:
        logging.error("Usage: python rescue_mission.py <directory_path>")
        sys.exit(1)

    # Directory path containing the .h5 files
    directory = sys.argv[1]

    # List of required keys
    required_keys = [
        'observations/images/oakd_front_view/projection',
        'observations/images/oakd_side_view/projection',
        'observations/images/oakd_front_view/intrinsics',
        'observations/images/oakd_side_view/intrinsics',
        'observations/images/oakd_front_view/extrinsics',
        'observations/images/oakd_side_view/extrinsics'
    ]

    # Check if the directory exists
    if os.path.isdir(directory):
        process_directory(directory, required_keys)
    else:
        logging.error(f"Error: The directory {directory} does not exist.")