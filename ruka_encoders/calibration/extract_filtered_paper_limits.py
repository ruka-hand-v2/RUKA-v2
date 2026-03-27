"""extract_filtered_paper_limits.py
Reads CSV files from the Paper folder and calculates the commanded extremums,
and applies a statistical median filter to the physical sensor output to
eliminate false high-frequency peaks (noise).
"""
import argparse
import glob
import json
import os

import pandas as pd
import numpy as np

# ── 0. Path Resolution ───────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
# Look for logs in the folder created by recalibrate_limits.py by default
DEFAULT_LOG_DIR = os.path.join(SCRIPT_DIR, "recalibrate_limits_logs")
# Default output mapping filename
DEFAULT_OUT_FILE = os.path.join(SCRIPT_DIR, "manually_filtered_paper_limits.json")

SENSOR_MOTOR_MAPPING = [
    {"sensor": 0, "motor": 8, "name": "Index Abduction"},
    {"sensor": 1, "motor": 7, "name": "Index DIP"},
    {"sensor": 2, "motor": 7, "name": "Index PIP"},
    {"sensor": 3, "motor": 9, "name": "Index MCP"},
    {"sensor": 4, "motor": 13, "name": "Thumb DIP"},
    {"sensor": 5, "motor": 14, "name": "Thumb MCP"},
    {"sensor": 6, "motor": 12, "name": "Thumb CMC"},
]

def main():
    parser = argparse.ArgumentParser(description="Filter real limits from CSV logs to ignore spikes/noise.")
    parser.add_argument("--dir", type=str, default=DEFAULT_LOG_DIR, help="Directory containing the recalibrate_limits CSV logs")
    parser.add_argument("--out", type=str, default=DEFAULT_OUT_FILE, help="Output JSON path")
    parser.add_argument("--window", type=int, default=5, help="Rolling median window size to completely crush 1-off noise peaks")
    args = parser.parse_args()

    # Search for all CSV files in target dir
    csv_files = glob.glob(os.path.join(args.dir, "*.csv"))
    if not csv_files:
        print(f"ERROR: No CSV files found in directory: {args.dir}")
        return

    print(f"Processing {len(csv_files)} log files found in '{args.dir}'...")

    # Concatenate them all
    df_list = [pd.read_csv(f) for f in csv_files]
    df = pd.concat(df_list, ignore_index=True)

    results = {}

    for mapping in SENSOR_MOTOR_MAPPING:
        s_idx = mapping["sensor"]
        m_id = mapping["motor"]
        arr_idx = m_id - 1
        name = mapping["name"]

        cmd_col = f"cmd_{arr_idx}"
        deg_col = f"deg_{s_idx}"

        if cmd_col not in df.columns or deg_col not in df.columns:
            continue

        # Look specifically at the phases where this joint was being isolated and tested.
        # This includes wallet, closed, open, and the manual phases specific to this sensor.
        phases = [f"S{s_idx}_wallet", f"S{s_idx}_open", f"S{s_idx}_manual", f"S{s_idx}_closed"]
        
        if "phase" in df.columns:
            phase_mask = df["phase"].isin(phases)
        else:
            phase_mask = pd.Series([True] * len(df))

        if not phase_mask.any():
            # If no specific phases found, fallback to the entire log
            phase_mask = pd.Series([True] * len(df))

        df_isolated = df[phase_mask]

        # 1. Commanded Max and Min
        cmd_vals = df_isolated[cmd_col].dropna()
        if not cmd_vals.empty:
            cmd_min = int(cmd_vals.min())
            cmd_max = int(cmd_vals.max())
        else:
            cmd_min = cmd_max = 0

        # 2. Filter the Sensor (eliminate artificial noise peaks)
        deg_vals = df_isolated[deg_col].dropna()
        if not deg_vals.empty:
            # Apply a rolling median filter. This entirely deletes isolated huge noise spikes 
            # (which last 1-2 packets) but accurately preserves the true maximum plateau 
            # achieved during physical bounds testing!
            smoothed_degs = deg_vals.rolling(window=args.window, min_periods=1, center=True).median()
            
            # Now safely take absolute physical limits from the cleaned array
            real_min = float(smoothed_degs.min())
            real_max = float(smoothed_degs.max())
            real_span = real_max - real_min
        else:
            real_min = real_max = real_span = 0.0

        results[str(s_idx)] = {
            "name": name,
            "motor_id": m_id,
            "commanded_min_ticks": cmd_min,
            "commanded_max_ticks": cmd_max,
            "filtered_sensor_min_deg": round(real_min, 4),
            "filtered_sensor_max_deg": round(real_max, 4),
            "filtered_sensor_span_deg": round(real_span, 4),
        }

    print("\n--- Processed Filtered Real Limits ---")
    print(json.dumps(results, indent=4))

    # Output to File
    with open(args.out, "w") as f:
        json.dump(results, f, indent=4)
        
    print(f"\nFiltered physics limits saved to -> {args.out}")

if __name__ == "__main__":
    main()
