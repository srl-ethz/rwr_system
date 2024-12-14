import h5py
import os
import sys

def print_h5_structure_if_missing(file_path):
    """Print the structure of an HDF5 file if specific keys are missing."""
    try:
        with h5py.File(file_path, 'r') as f:
            missing_keys = []
            required_keys = [
                'observations/images/oakd_front_view/projection',
                'observations/images/oakd_side_view/projection',
                'observations/images/oakd_front_view/intrinsics',
                'observations/images/oakd_side_view/intrinsics',
                'observations/images/oakd_front_view/extrinsics',
                'observations/images/oakd_side_view/extrinsics'
            ]
            
            for key in required_keys:
                if key not in f:
                    missing_keys.append(key)
            
            if missing_keys:
                print(f"Missing keys in {file_path}: {missing_keys}")
    except Exception as e:
        print(f"Error reading {file_path}: {e}")

def process_directory(directory):
    """Process all .h5 files in a directory."""
    # Loop through all files in the directory
    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        
        # Only process .h5 files
        if file_path.endswith('.h5') and os.path.isfile(file_path):
            print_h5_structure_if_missing(file_path)

if __name__ == "__main__":
    # Check if directory path is provided as a command line argument
    if len(sys.argv) != 2:
        print("Usage: python check_h5_file.py <directory_path>")
        sys.exit(1)

    # Directory path containing the .h5 files
    directory = sys.argv[1]

    # Check if the directory exists
    if os.path.isdir(directory):
        process_directory(directory)
    else:
        print(f"Error: The directory {directory} does not exist.")