import pandas as pd
import matplotlib.pyplot as plt
import argparse
import numpy as np
from scipy import signal
from scipy.fft import fft, fftfreq
import os

# Parse command line arguments
parser = argparse.ArgumentParser(description='Plot chirp data from CSV file')
parser.add_argument('csv_path', help='Path to the CSV file')
parser.add_argument('--delay-steps', type=int, default=0, help='Number of samples to delay the measured signal (positive = measured lags, negative = measured leads)')
parser.add_argument('--trim-first-seconds', type=float, default=0.1, help='Seconds to discard before time alignment and frequency analysis (default: 0.1)')
parser.add_argument('--save', action='store_true', help='Save plot to PNG file instead of showing')
args = parser.parse_args()
if not np.isfinite(args.trim_first_seconds) or args.trim_first_seconds < 0:
    parser.error('--trim-first-seconds must be finite and non-negative')

# Load the CSV
df = pd.read_csv(args.csv_path)

# Extract filename for title
csv_filename = os.path.basename(args.csv_path)

# Original time series plot
plt.figure(figsize=(15, 12))

# Add main title with CSV filename
plt.suptitle(f'Chirp Analysis: {csv_filename}', fontsize=16, y=0.98)

plt.subplot(3, 2, 1)
plt.plot(df['timestamp'], df['commanded_position'], label='Commanded Position')
plt.plot(df['timestamp'], df['measured_position'], label='Measured Position')
plt.xlabel('Timestamp [s]')
plt.ylabel('Position')
plt.title('Original Signals')
plt.legend()
plt.grid(True)

# Step 1: Trim first seconds to get rid of transient effects
trim_first_seconds = args.trim_first_seconds
trim_mask = df['timestamp'] >= trim_first_seconds
df_trimmed = df[trim_mask].copy()
if len(df_trimmed) - abs(args.delay_steps) < 3:
    parser.error('Too few samples remain after trimming and time alignment')
df_trimmed['timestamp'] = df_trimmed['timestamp'] - df_trimmed['timestamp'].iloc[0]

plt.subplot(3, 2, 2)
plt.plot(df_trimmed['timestamp'], df_trimmed['commanded_position'], label='Commanded Position')
plt.plot(df_trimmed['timestamp'], df_trimmed['measured_position'], label='Measured Position')
plt.xlabel('Timestamp [s]')
plt.ylabel('Position')
plt.title(f'After Trimming First {trim_first_seconds} seconds')
plt.legend()
plt.grid(True)

# Step 2: Time alignment using user-specified delay
cmd_pos = df_trimmed['commanded_position'].values
meas_pos = df_trimmed['measured_position'].values

delay_samples = args.delay_steps

# Apply time shift based on user-specified delay
if delay_samples > 0:
    meas_pos_aligned = meas_pos[delay_samples:]
    cmd_pos_aligned = cmd_pos[:len(meas_pos_aligned)]
elif delay_samples < 0:
    cmd_pos_aligned = cmd_pos[-delay_samples:]
    meas_pos_aligned = meas_pos[:len(cmd_pos_aligned)]
else:
    cmd_pos_aligned = cmd_pos
    meas_pos_aligned = meas_pos

time_aligned = df_trimmed['timestamp'].values[:len(cmd_pos_aligned)]
title_suffix = f' (Delay: {delay_samples} samples)'

plt.subplot(3, 2, 3)
plt.plot(time_aligned, cmd_pos_aligned, label='Commanded Position')
plt.plot(time_aligned, meas_pos_aligned, label='Measured Position')
plt.xlabel('Timestamp [s]')
plt.ylabel('Position')
plt.title('After Time Alignment' + title_suffix)
plt.legend()
plt.grid(True)

# Step 3: Subtract mean and apply Tukey window
cmd_pos_centered = cmd_pos_aligned - np.mean(cmd_pos_aligned)
meas_pos_centered = meas_pos_aligned - np.mean(meas_pos_aligned)

# Apply Tukey window (10% taper)
window = signal.windows.tukey(len(cmd_pos_centered), alpha=0.1)
cmd_pos_windowed = cmd_pos_centered * window
meas_pos_windowed = meas_pos_centered * window

plt.subplot(3, 2, 4)
plt.plot(time_aligned, cmd_pos_windowed, label='Commanded Position')
plt.plot(time_aligned, meas_pos_windowed, label='Measured Position')
plt.plot(time_aligned, window * np.max(np.abs(cmd_pos_windowed)), 'k--', alpha=0.5, label='Tukey Window')
plt.xlabel('Timestamp [s]')
plt.ylabel('Position')
plt.title('After Mean Removal and Windowing')
plt.legend()
plt.grid(True)

# Step 4: Compute Frequency Response Function
dt = np.mean(np.diff(time_aligned))
fs = 1.0 / dt

# FFT of input and output
cmd_fft = fft(cmd_pos_windowed)
meas_fft = fft(meas_pos_windowed)

# Frequency vector
freqs = fftfreq(len(cmd_pos_windowed), dt)
positive_freq_mask = freqs > 0

# Filter frequencies to between the chirp signal to reduce noise
MIN_FREQ = 0.1  # Hz
MAX_FREQ = 5.0  # Hz
print(f"Frequency range: {MIN_FREQ} Hz to {MAX_FREQ} Hz")
freq_limit_mask = (freqs <= MAX_FREQ) & (freqs >= MIN_FREQ)
combined_mask = positive_freq_mask & freq_limit_mask

freqs = freqs[combined_mask]
# Frequency Response Function
H = meas_fft[combined_mask] / cmd_fft[combined_mask]

# Magnitude and phase
magnitude_db = 20 * np.log10(np.abs(H))
# Unwrap within the selected frequency band to remove 360-degree phase jumps.
phase_deg = np.rad2deg(np.unwrap(np.angle(H)))

# Bode plot
plt.subplot(3, 2, 5)
plt.semilogx(freqs, magnitude_db)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude [dB]')
plt.title('Bode Plot - Magnitude')
plt.grid(True)

plt.subplot(3, 2, 6)
plt.semilogx(freqs, phase_deg)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Phase [degrees]')
plt.title('Bode Plot - Phase (unwrapped)')
plt.grid(True)

plt.tight_layout()

if args.save:
    # Extract base name from CSV path and create PNG filename
    base_name = os.path.splitext(os.path.basename(args.csv_path))[0]
    output_path = f"{base_name}.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
else:
    plt.show()
