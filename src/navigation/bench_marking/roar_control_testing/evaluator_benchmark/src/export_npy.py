#!/usr/bin/env python3
"""
Exporter script to convert evaluator_control CSV telemetry into NumPy format (.npy & .npz).
"""
import os
import sys
import numpy as np

def convert_csv_to_npy(csv_path="/tmp/controller_global_behavior.csv", 
                       npy_output="/tmp/evaluator_control_data.npy",
                       npz_output="/tmp/evaluator_control_data.npz"):
    if not os.path.exists(csv_path):
        print(f"[ERROR] Input CSV file does not exist: {csv_path}")
        return False

    print(f"[INFO] Reading CSV telemetry from: {csv_path}")

    # Define custom structured dtype for .npy file
    dtype_spec = [
        ('timestamp_s', 'f8'),
        ('case_id', 'U32'),
        ('cmd_linear_v', 'f4'),
        ('actual_linear_v', 'f4'),
        ('cmd_angular_w', 'f4'),
        ('actual_angular_w', 'f4'),
        ('linear_jerk_m_s3', 'f4'),
        ('steering_jerk_deg_s2', 'f4'),
        ('lat_error_m', 'f4'),
        ('pos_error_m', 'f4'),
        ('heading_error_deg', 'f4'),
        ('overshoot_m', 'f4'),
        ('linear_violation', 'i4'),
        ('angular_violation', 'i4'),
        ('jerk_instability', 'i4'),
        ('similarity_ratio', 'f4')
    ]

    try:
        # Load structured array using numpy genfromtxt
        data_struct = np.genfromtxt(
            csv_path,
            delimiter=',',
            names=True,
            dtype=dtype_spec,
            encoding='utf-8'
        )

        num_samples = len(data_struct)
        print(f"[INFO] Successfully loaded {num_samples} telemetry samples.")

        # 1. Save Structured Array to .npy
        np.save(npy_output, data_struct)
        print(f"[SUCCESS] Structured NumPy array saved to: {npy_output} ({os.path.getsize(npy_output) / 1024:.2f} KB)")

        # 2. Save Dictionary of Individual Column Arrays to .npz (Compressed)
        npz_dict = {name: data_struct[name] for name in data_struct.dtype.names}
        np.savez_compressed(npz_output, **npz_dict)
        print(f"[SUCCESS] Compressed NumPy archive saved to: {npz_output} ({os.path.getsize(npz_output) / 1024:.2f} KB)")

        return True

    except Exception as e:
        print(f"[ERROR] Failed to convert CSV to NumPy format: {e}")
        return False

if __name__ == '__main__':
    csv_file = sys.argv[1] if len(sys.argv) > 1 else "/tmp/controller_global_behavior.csv"
    convert_csv_to_npy(csv_file)
