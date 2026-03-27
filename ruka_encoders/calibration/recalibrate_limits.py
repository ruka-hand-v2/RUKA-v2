"""recalibrate_limits.py
========================
Automated and manual sequence script to capture true limits of each sensor.

Workflow per sensor/motor:
1. Command hand to OPEN_POS.
2. Move specific motor to its WALLET_POS equivalent.
3. Return specific motor to OPEN_POS.
4. Wait for user to MANUALLY move the finger to capture ultimate physical limits.

Outputs:
- A unified CSV log containing cmd, act, calculated angle, and sensor readings.
- A JSON file containing the actual min/max degrees achieved during the manual limits test.
- Plots (similar to analyze_complete_logs.py) to visually validate noise vs movement.
"""

import argparse
import csv
import json
import math
import os
import queue
import select
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import re

# ── 0. Path Resolution ───────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Default output directory relative to this script
DEFAULT_OUT_DIR = os.path.join(SCRIPT_DIR, "recalibrate_limits_logs")

import matplotlib.pyplot as plt
import numpy as np
import serial

try:
    from ruka_hand.control.hand import Hand
except ImportError:
    Hand = None

N_SENSORS = 7
N_MOTORS = 16

# -------------------------------------------------------------------------
# Customizable Configurations
# -------------------------------------------------------------------------

# The target arrays (from replay_encoders.py). Modify these if needed.
OPEN_POS = [3559, 2864, 2247, 1891, 3407, 849, 2983, 1641, 1249, 853, 230, 1060, 3042, 2288, 2000, 1850]

# Single array storing all max closed limits. 
# WARNING: NEVER send this whole array to the hand at once! The physical fingers will collide.
# The script natively correctly extracts only one motor at a time.
CLOSED_POS = OPEN_POS.copy()
CLOSED_POS[6] = 1600   # M7  (Index PIP/DIP)
CLOSED_POS[7] = 1250   # M8  (Index Abduction)
CLOSED_POS[8] = 592    # M9  (Index MCP)
CLOSED_POS[11] = 492   # M12 (Thumb CMC)
CLOSED_POS[12] = 2000  # M13 (Thumb DIP)
CLOSED_POS[13] = 1350  # M14 (Thumb MCP)

# Mapping of Sensor to Motor ID
SENSOR_MOTOR_MAPPING = [
    {"sensor": 0, "motor": 8, "name": "Index Abduction"},
    {"sensor": 1, "motor": 7, "name": "Index DIP"},
    {"sensor": 2, "motor": 7, "name": "Index PIP"},
    {"sensor": 3, "motor": 9, "name": "Index MCP"},
    {"sensor": 4, "motor": 13, "name": "Thumb DIP"},
    {"sensor": 5, "motor": 14, "name": "Thumb MCP"},
    {"sensor": 6, "motor": 12, "name": "Thumb CMC"},
]

# -------------------------------------------------------------------------
# Math & Logic Helpers
# -------------------------------------------------------------------------

MIN_DEG = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -25, 0], dtype=float)
MAX_DEG = np.array([90, 40, 85, 15, 90, 85, 70, 20, 90, 80, 90, 90, 145, 90, 25, 60], dtype=float)

def _solve_quad_for_motor8(clamped_val: float) -> float:
    if not np.isfinite(clamped_val):
        return float("nan")
    disc = 1.0 + 0.08 * max(clamped_val, 0.0)
    return float((-1.0 + np.sqrt(disc)) / 0.04)

def estimate_joint_angle_from_tick(hand: Hand, motor_idx: int, tick: float) -> float:
    if hand is None or not np.isfinite(tick):
        return float("nan")

    try:
        tensioned = np.array(hand.tensioned_pos, dtype=float)
        curled = np.array(hand.curled_bound, dtype=float)
        span_abs = abs(curled[motor_idx] - tensioned[motor_idx])

        if motor_idx == 1:
            normed = (tick - 2285.0) / span_abs if span_abs > 1e-9 else np.nan
        elif motor_idx == 3:
            normed = (2070.0 - tick) / span_abs if span_abs > 1e-9 else np.nan
        elif motor_idx == 7:
            normed = (tick - 2125.0) / span_abs if span_abs > 1e-9 else np.nan
        elif motor_idx == 14:
            normed = (tick - 1990.0) / span_abs if span_abs > 1e-9 else np.nan
        else:
            span = curled[motor_idx] - tensioned[motor_idx]
            normed = (tick - tensioned[motor_idx]) / span if abs(span) > 1e-9 else np.nan

        if not np.isfinite(normed):
            return float("nan")

        clamped_val = normed * (MAX_DEG[motor_idx] - MIN_DEG[motor_idx])

        if motor_idx == 8:
            return _solve_quad_for_motor8(clamped_val)
        if motor_idx == 12:
            return clamped_val / 1.8
        if motor_idx == 13:
            return clamped_val / 1.5 + 40.0
        if motor_idx == 7:
            return clamped_val / 1.7
        if motor_idx == 3:
            return clamped_val / 1.7
        if motor_idx == 1:
            return clamped_val / 1.7

        return clamped_val
    except Exception:
        return float("nan")

def _parse_sensor_line(line: str) -> dict[int, dict[str, float | int]] | None:
    line = line.strip()
    if not line or line.startswith("["):
        return None
    out = {}
    matches_with_raw = re.findall(r"S(\d+)\s*:\s*([-+]?\d*\.?\d+)\s*:\s*(-?\d+)", line)
    for s_str, deg_str, raw_str in matches_with_raw:
        out[int(s_str)] = {"deg": float(deg_str), "raw": int(raw_str)}
    if out:
        return out
    matches_deg_only = re.findall(r"Sensor(\d+)\s*:\s*([-+]?\d*\.?\d+)", line)
    for s_str, deg_str in matches_deg_only:
        out[int(s_str)] = {"deg": float(deg_str), "raw": -1}
    return out if out else None

# -------------------------------------------------------------------------
# Logger Thread
# -------------------------------------------------------------------------

class CalibrationLogger:
    def __init__(self, serial_port: str, baud: int, hand: Hand, hand_io_lock: threading.Lock):
        self.conn = serial.Serial(serial_port, baud, timeout=0.2)
        time.sleep(1.0)
        self.conn.reset_input_buffer()
        self.hand = hand
        self.hand_io_lock = hand_io_lock
        
        self.stop_event = threading.Event()
        self.log_data = []
        self.t0 = 0.0
        
        # Current commanded and phase tracked safely
        self.lock = threading.Lock()
        self.current_cmd = OPEN_POS.copy()
        self.current_phase = "init"
        
        self.thread = threading.Thread(target=self._loop, daemon=True)
        
    def start(self):
        self.t0 = time.time()
        self.conn.write(b"s") # ensure streaming
        self.thread.start()
        
    def stop(self):
        self.stop_event.set()
        self.thread.join()
        try:
            self.conn.write(b"s")
            self.conn.close()
        except Exception:
            pass
            
    def set_command_state(self, cmd_pos, phase_name):
        with self.lock:
            self.current_cmd = cmd_pos.copy()
            self.current_phase = phase_name
            
    def _loop(self):
        while not self.stop_event.is_set():
            try:
                line = self.conn.readline().decode(errors="ignore")
                parsed = _parse_sensor_line(line)
            except Exception:
                parsed = None
                
            if parsed is None:
                continue
                
            t_rel = time.time() - self.t0
            
            # Read motor ACTUAL positions
            actual_pos = [-1] * N_MOTORS
            if self.hand is not None:
                try:
                    with self.hand_io_lock:
                        p = self.hand.read_pos()
                    if p:
                        actual_pos = [int(x) if x is not None else -1 for x in p[:N_MOTORS]]
                except Exception:
                    pass
            
            with self.lock:
                cmd_pos = self.current_cmd.copy()
                phase = self.current_phase
            
            # Calculate angles
            act_angles = []
            cmd_angles = []
            for i in range(N_MOTORS):
                act_angles.append(estimate_joint_angle_from_tick(self.hand, i, actual_pos[i]))
                cmd_angles.append(estimate_joint_angle_from_tick(self.hand, i, cmd_pos[i]))
                
            # Parse Sensors
            raw_vals = [-1] * N_SENSORS
            deg_vals = [float("nan")] * N_SENSORS
            for s in range(N_SENSORS):
                if s in parsed:
                    raw_vals[s] = parsed[s].get("raw", -1)
                    deg_vals[s] = parsed[s]["deg"]
                    
            self.log_data.append({
                "t_rel": t_rel,
                "phase": phase,
                "commanded": cmd_pos,
                "actual": actual_pos,
                "cmd_angles": cmd_angles,
                "act_angles": act_angles,
                "raw": raw_vals,
                "deg": deg_vals
            })

# -------------------------------------------------------------------------
# Movement interpolation
# -------------------------------------------------------------------------
def move_interpolated(hand, hand_io_lock, start_pos, end_pos, steps=40, sleep_dt=0.01):
    for i in range(1, steps + 1):
        alpha = i / float(steps)
        cmd = [int(start_pos[j] * (1 - alpha) + end_pos[j] * alpha) for j in range(len(start_pos))]
        if hand is not None:
            with hand_io_lock:
                hand.set_pos(cmd)
        time.sleep(sleep_dt)

# -------------------------------------------------------------------------
# Main Execution
# -------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Recalibrate limits manually with sequential motor commands.")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hand", type=str, default="right", choices=["left", "right"])
    parser.add_argument("--out-dir", type=str, default=DEFAULT_OUT_DIR)
    args = parser.parse_args()

    if Hand is None:
        print("ERROR: ruka_hand package not found. Cannot control motors.")
        sys.exit(1)

    print(f"Connecting to {args.hand} hand...")
    try:
        hand = Hand(args.hand)
    except Exception as e:
        print(f"Failed to connect to hand: {e}")
        sys.exit(1)
    
    hand_io_lock = threading.Lock()
    
    # Init logger
    logger = CalibrationLogger(args.serial_port, args.baud, hand, hand_io_lock)
    logger.start()
    
    try:
        print("Moving to OPEN hand position...")
        logger.set_command_state(OPEN_POS, "init_open")
        with hand_io_lock:
            current_pos = hand.read_pos()
            if current_pos:
                current_pos = [int(p) if p is not None else OPEN_POS[i] for i, p in enumerate(current_pos[:N_MOTORS])]
            else:
                current_pos = OPEN_POS.copy()
                
        move_interpolated(hand, hand_io_lock, current_pos, OPEN_POS, steps=50)
        time.sleep(1.0)
        
        current_pos = OPEN_POS.copy()
        
        for mapping in SENSOR_MOTOR_MAPPING:
            s_idx = mapping["sensor"]
            m_id = mapping["motor"]
            arr_idx = m_id - 1
            name = mapping["name"]
            
            print(f"\n{'='*60}")
            print(f"Testing Sensor {s_idx} / Motor {m_id} ({name})")
            print(f"{'='*60}")
            
            # Action: Target Closed Pos for this specific motor
            target_pos = OPEN_POS.copy()
            target_pos[arr_idx] = CLOSED_POS[arr_idx]
            
            # Provide space for manually moving safely
            return_pos = OPEN_POS.copy()
            if s_idx == 0:
                target_pos[9] = 1700
                return_pos[9] = 1700
            elif s_idx == 3:
                # To make space for Index MCP (Motor 9), 
                # fully close Index Abduction (Motor 8 / arr_idx 7)
                target_pos[7] = CLOSED_POS[7]
                return_pos[7] = CLOSED_POS[7]
            
            print(f"  -> Moving Motor {m_id} to Maximum/Closed position...")
            logger.set_command_state(target_pos, f"S{s_idx}_closed")
            move_interpolated(hand, hand_io_lock, current_pos, target_pos, steps=40)
            time.sleep(1.0)
            
            # Action: Return to Open Pos
            print(f"  -> Returning Motor {m_id} to Open position...")
            logger.set_command_state(return_pos, f"S{s_idx}_open")
            move_interpolated(hand, hand_io_lock, target_pos, return_pos, steps=40)
            time.sleep(1.0)
            
            # Action: Manual Limits!
            logger.set_command_state(return_pos, f"S{s_idx}_manual")
            input(f"\n  >>> NOW manually move the {name} (Sensor {s_idx}) to its physical limits. Press Enter when done...")
            
            current_pos = return_pos.copy()
            
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nStopping logger and saving data...")
        logger.stop()
        if hand is not None:
            with hand_io_lock:
                hand.close()
        
    # Process Logs & JSON Output
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 1. Save CSV
    csv_path = Path(args.out_dir) / f"{stamp}_limits_recalibration_log.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        header = ["t_rel", "phase"]
        header += [f"raw_{i}" for i in range(N_SENSORS)]
        header += [f"deg_{i}" for i in range(N_SENSORS)]
        header += [f"cmd_{i}" for i in range(N_MOTORS)]
        header += [f"act_{i}" for i in range(N_MOTORS)]
        header += [f"ang_cmd_{i}" for i in range(N_MOTORS)]
        header += [f"ang_act_{i}" for i in range(N_MOTORS)]
        w.writerow(header)
        
        for row in logger.log_data:
            line = [row["t_rel"], row["phase"]]
            line += row["raw"]
            line += [f"{d:.2f}" if np.isfinite(d) else "nan" for d in row["deg"]]
            line += row["commanded"]
            line += row["actual"]
            line += [f"{d:.2f}" if np.isfinite(d) else "nan" for d in row["cmd_angles"]]
            line += [f"{d:.2f}" if np.isfinite(d) else "nan" for d in row["act_angles"]]
            w.writerow(line)
            
    print(f"Saved complete unified log to: {csv_path}")
    
    # 2. Extract Manual Limits to JSON
    extracted_limits = {}
    
    # Initialize all 7 sensors to avoid missing json keys if interrupted
    for s in range(N_SENSORS):
        extracted_limits[str(s)] = {
            "min_deg": None,
            "max_deg": None,
            "span": None,
            "min_motor_ticks": None,
            "max_motor_ticks": None
        }

    for mapping in SENSOR_MOTOR_MAPPING:
        s_idx = mapping["sensor"]
        m_id = mapping["motor"]
        arr_idx = m_id - 1
        
        phase_name = f"S{s_idx}_manual"
        phase_rows = [r for r in logger.log_data if r["phase"] == phase_name]
        
        # Get sensor degrees
        degs = [r["deg"][s_idx] for r in phase_rows if np.isfinite(r["deg"][s_idx])]
        # Get actual motor ticks
        m_ticks = [r["actual"][arr_idx] for r in phase_rows if r["actual"][arr_idx] != -1]
        
        # Fallback to the entire log sequence if the manual phase was skipped/interrupted
        if not degs:
            degs = [r["deg"][s_idx] for r in logger.log_data if np.isfinite(r["deg"][s_idx])]
        if not m_ticks:
            m_ticks = [r["actual"][arr_idx] for r in logger.log_data if r["actual"][arr_idx] != -1]
            
        if degs:
            extracted_limits[str(s_idx)]["min_deg"] = min(degs)
            extracted_limits[str(s_idx)]["max_deg"] = max(degs)
            extracted_limits[str(s_idx)]["span"] = max(degs) - min(degs)
            
        if m_ticks:
            extracted_limits[str(s_idx)]["min_motor_ticks"] = min(m_ticks)
            extracted_limits[str(s_idx)]["max_motor_ticks"] = max(m_ticks)
            
    json_path = Path(args.out_dir) / f"{stamp}_manual_sensor_limits.json"
    with json_path.open("w") as f:
        json.dump(extracted_limits, f, indent=4)
    print(f"Saved extracted manual limits to: {json_path}")

    # 3. Plot Graphs using matplotlib (Cmd vs Act vs Sensor)
    print("Generating validation plots...")
    try:
        import pandas as pd
        df = pd.read_csv(csv_path)
        
        num_subplots = len(SENSOR_MOTOR_MAPPING)
        fig, axes = plt.subplots(num_subplots, 1, figsize=(14, 4 * num_subplots), sharex=True)
        if num_subplots == 1:
            axes = [axes]
            
        for idx, mapping in enumerate(SENSOR_MOTOR_MAPPING):
            ax = axes[idx]
            s_idx = mapping["sensor"]
            m_id = mapping["motor"]
            arr_idx = m_id - 1
            name = mapping["name"]
            
            c_sen = f"deg_{s_idx}"
            c_ang = f"ang_act_{arr_idx}"
            
            if c_sen in df.columns:
                ax.plot(df["t_rel"], df[c_sen], label=f"Sensor {s_idx} ({name})", color="darkorange", lw=2)
            if c_ang in df.columns:
                ax.plot(df["t_rel"], df[c_ang], label=f"Calculated Angle M{m_id}", color="green", ls="--", lw=1.5)
                
            # Shade regions where manual testing occurred
            phase_starts = df.index[df['phase'] == f"S{s_idx}_manual"].tolist()
            if phase_starts:
                t_starts = df.iloc[phase_starts]["t_rel"]
                ax.axvspan(t_starts.min(), t_starts.max(), color='yellow', alpha=0.2, label='Manual Testing Phase')

            ax.set_title(f"Sensor {s_idx} | Motor {m_id} ({name})", fontweight='bold')
            ax.grid(alpha=0.3)
            ax.legend(loc="upper right")
            
        axes[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plot_path = Path(args.out_dir) / f"{stamp}_calibration_plot.png"
        plt.savefig(plot_path, dpi=150)
        print(f"Saved plot to: {plot_path}")
        
    except Exception as e:
        print(f"Plotting failed: {e}")

if __name__ == "__main__":
    main()
