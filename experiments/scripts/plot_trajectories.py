#!/usr/bin/env python3
"""
Plot robotic trajectory data from HDF5 files.
Creates two types of plots:
1. For each dimension k: all runs on one figure (to grasp overall trends)
2. For each run: all dimensions of the observation key
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path
import re

def natural_sort_key(s):
    """Natural sorting for step names (step_0, step_1, ..., step_10, ...)"""
    return [int(text) if text.isdigit() else text.lower() 
            for text in re.split('([0-9]+)', str(s))]

def read_trajectory_data(hdf5_path, obs_key):
    """
    Read trajectory data for a specific observation key from HDF5 file.
    
    Args:
        hdf5_path: Path to HDF5 file
        obs_key: Observation key (e.g., 'qpos_arm', 'qpos_hand')
    
    Returns:
        numpy array of shape (num_steps, num_dimensions) or None if no data
    """
    with h5py.File(hdf5_path, 'r') as f:
        # Get all step keys and sort them naturally
        step_keys = [key for key in f.keys() if key.startswith('step_')]
        
        if not step_keys:
            return None
        
        step_keys = sorted(step_keys, key=natural_sort_key)
        
        # Read data from each step
        data_list = []
        for step_key in step_keys:
            obs_data = f[step_key]['observations'][obs_key][:]
            data_list.append(obs_data)
        
        if not data_list:
            return None
            
        return np.array(data_list)

def plot_dimension_across_runs(subdirs, obs_key, output_dir):
    """
    Plot 1: For each dimension k, plot all runs on one figure.
    
    Args:
        subdirs: List of subdirectory paths
        obs_key: Observation key ('qpos_arm' or 'qpos_hand')
        output_dir: Directory to save plots
    """
    print(f"\nCreating dimension-wise plots for {obs_key}...")
    
    for subdir in subdirs:
        subdir_path = Path(subdir)
        hdf5_files = sorted(list(subdir_path.glob('*.hdf5')))
        
        if not hdf5_files:
            print(f"  No HDF5 files found in {subdir_path.name}")
            continue
        
        print(f"  Processing {subdir_path.name} ({len(hdf5_files)} files)")
        
        # Read all trajectories
        all_trajectories = []
        run_names = []
        for hdf5_file in hdf5_files:
            try:
                traj_data = read_trajectory_data(hdf5_file, obs_key)
                if traj_data is not None and len(traj_data) > 0:
                    all_trajectories.append(traj_data)
                    run_names.append(hdf5_file.stem)
                else:
                    print(f"    Skipping {hdf5_file.name}: No data")
            except Exception as e:
                print(f"    Warning: Could not read {hdf5_file.name}: {e}")
        
        if not all_trajectories:
            print(f"    No valid trajectories found for {obs_key}")
            continue
        
        # Get number of dimensions
        num_dims = all_trajectories[0].shape[1]
        
        # Create output directory for this subdirectory and obs_key
        plot_output_dir = Path(output_dir) / subdir_path.name / f"{obs_key}_by_dimension"
        plot_output_dir.mkdir(parents=True, exist_ok=True)
        
        # Plot each dimension
        for dim_idx in range(num_dims):
            fig, ax = plt.subplots(figsize=(12, 6))
            
            # Plot all runs for this dimension
            for traj, run_name in zip(all_trajectories, run_names):
                time_steps = np.arange(len(traj))
                ax.plot(time_steps, traj[:, dim_idx], label=run_name, alpha=0.7)
            
            ax.set_xlabel('Time Step', fontsize=12)
            ax.set_ylabel(f'{obs_key} Value', fontsize=12)
            ax.set_title(f'{obs_key} - Dimension {dim_idx} - All Runs\n({subdir_path.name})', 
                        fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
            
            plt.tight_layout()
            output_file = plot_output_dir / f"{obs_key}_dim_{dim_idx}.png"
            plt.savefig(output_file, dpi=150, bbox_inches='tight')
            plt.close()
        
        print(f"    Saved {num_dims} dimension plots to {plot_output_dir}")

def plot_all_dimensions_per_run(subdirs, obs_key, output_dir):
    """
    Plot 2: For each run, plot all dimensions of the observation key.
    
    Args:
        subdirs: List of subdirectory paths
        obs_key: Observation key ('qpos_arm' or 'qpos_hand')
        output_dir: Directory to save plots
    """
    print(f"\nCreating per-run plots for {obs_key}...")
    
    for subdir in subdirs:
        subdir_path = Path(subdir)
        hdf5_files = sorted(list(subdir_path.glob('*.hdf5')))
        
        if not hdf5_files:
            continue
        
        print(f"  Processing {subdir_path.name} ({len(hdf5_files)} files)")
        
        # Create output directory for this subdirectory and obs_key
        plot_output_dir = Path(output_dir) / subdir_path.name / f"{obs_key}_by_run"
        plot_output_dir.mkdir(parents=True, exist_ok=True)
        
        # Plot each run
        for hdf5_file in hdf5_files:
            try:
                traj_data = read_trajectory_data(hdf5_file, obs_key)
                
                if traj_data is None or len(traj_data) == 0:
                    print(f"    Skipping {hdf5_file.name}: No data")
                    continue
                    
                run_name = hdf5_file.stem
                
                num_dims = traj_data.shape[1]
                time_steps = np.arange(len(traj_data))
                
                # Create figure with subplots
                fig, axes = plt.subplots(num_dims, 1, figsize=(12, 2*num_dims), sharex=True)
                
                # Handle single dimension case
                if num_dims == 1:
                    axes = [axes]
                
                for dim_idx in range(num_dims):
                    axes[dim_idx].plot(time_steps, traj_data[:, dim_idx], linewidth=2)
                    axes[dim_idx].set_ylabel(f'Dim {dim_idx}', fontsize=10)
                    axes[dim_idx].grid(True, alpha=0.3)
                
                axes[-1].set_xlabel('Time Step', fontsize=12)
                fig.suptitle(f'{obs_key} - All Dimensions - {run_name}\n({subdir_path.name})', 
                            fontsize=14, fontweight='bold')
                
                plt.tight_layout()
                output_file = plot_output_dir / f"{obs_key}_{run_name}.png"
                plt.savefig(output_file, dpi=150, bbox_inches='tight')
                plt.close()
                
            except Exception as e:
                print(f"    Warning: Could not process {hdf5_file.name}: {e}")
        
        print(f"    Saved {len(hdf5_files)} run plots to {plot_output_dir}")

def main():
    # Get current directory
    base_dir = Path(os.path.join(Path.cwd(),  "inference_logs"))
    
    # Find all subdirectories containing HDF5 files
    subdirs = [d for d in base_dir.iterdir() if d.is_dir() and list(d.glob('*.hdf5'))]
    
    if not subdirs:
        print("No subdirectories with HDF5 files found!")
        return
    
    print(f"Found {len(subdirs)} subdirectories with HDF5 files:")
    for subdir in subdirs:
        num_files = len(list(subdir.glob('*.hdf5')))
        print(f"  - {subdir.name}: {num_files} files")
    
    # Create output directory
    output_dir = base_dir / "plots"
    output_dir.mkdir(exist_ok=True)
    
    # Observation keys to plot
    obs_keys = ['qpos_arm', 'qpos_hand']
    
    # Generate plots
    for obs_key in obs_keys:
        # Plot 1: Each dimension with all runs
        plot_dimension_across_runs(subdirs, obs_key, output_dir)
        
        # Plot 2: Each run with all dimensions
        plot_all_dimensions_per_run(subdirs, obs_key, output_dir)
    
    print(f"\n{'='*60}")
    print(f"All plots saved to: {output_dir}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
