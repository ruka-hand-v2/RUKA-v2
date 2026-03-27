"""random_joint_generator.py
==========================
Loads the strictly filtered paper limits JSON, bounds the hand movements,
and executes sequential random commanded positions one joint at a time.

Usage Example:
--------------
To test every joint sequentially with 5 random points each, 
waiting 8.0 seconds between taking a reading, run:

    python random_joint_generator.py --points-per-joint 5 --wait-time 8.0

(If left blank, it defaults to 5 points and 5.0 seconds).
"""

import argparse
import csv
import json
import os
import random
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── 0. Path Resolution ───────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
# Find project root (one level up from data_collection)
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
# Path to the shared calibration JSON
DEFAULT_JSON_PATH = os.path.join(PROJECT_ROOT, "calibration", "manually_filtered_paper_limits.json")
# Default output directory relative to this script
DEFAULT_OUT_DIR = os.path.join(SCRIPT_DIR, "random_generator_logs")

import numpy as np
import serial

try:
    from ruka_hand.control.hand import Hand
except ImportError:
    Hand = None

N_SENSORS = 7
N_MOTORS = 16

OPEN_POS = [3559, 2864, 2247, 1891, 3407, 849, 2983, 1641, 1249, 853, 230, 1060, 3042, 2288, 2000, 1850]

# -------------------------------------------------------------------------
# Interpolation Math
# -------------------------------------------------------------------------
def expected_angle_from_json(limits_map: dict, motor_id: int, tick: float) -> float:
    if not np.isfinite(tick):
        return float("nan")
        
    for s_idx, config in limits_map.items():
        if config["motor_id"] == motor_id:
            c_min = float(config["commanded_min_ticks"])
            c_max = float(config["commanded_max_ticks"])
            d_min = float(config["filtered_sensor_min_deg"])
            d_max = float(config["filtered_sensor_max_deg"])
            
            span_ticks = c_max - c_min
            if span_ticks == 0:
                return float("nan")
                
            # Linear interpolation matching the bounds strictly defined in the JSON
            # Handles inversions automatically based on the mapping!
            norm = (float(tick) - c_min) / span_ticks
            return d_min + norm * (d_max - d_min)

    # Not found in JSON model
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
# Thread-safe Logger
# -------------------------------------------------------------------------
class RandomSequenceLogger:
    def __init__(self, serial_port: str, baud: int, hand: Hand, io_lock: threading.Lock, limits: dict):
        self.conn = serial.Serial(serial_port, baud, timeout=0.2)
        time.sleep(1.0)
        self.conn.reset_input_buffer()
        self.hand = hand
        self.io_lock = io_lock
        self.limits = limits
        
        self.stop_event = threading.Event()
        self.log_data = []
        self.t0 = 0.0
        
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
            
            actual_pos = [-1] * N_MOTORS
            if self.hand is not None:
                try:
                    with self.io_lock:
                        p = self.hand.read_pos()
                    if p:
                        actual_pos = [int(x) if x is not None else -1 for x in p[:N_MOTORS]]
                except Exception:
                    pass
            
            with self.lock:
                cmd_pos = self.current_cmd.copy()
                phase = self.current_phase
            
            # Predict angles perfectly derived right off the JSON boundaries
            cmd_angles = []
            act_angles = []
            for i in range(N_MOTORS):
                m_id = i + 1
                cmd_angles.append(expected_angle_from_json(self.limits, m_id, cmd_pos[i]))
                act_angles.append(expected_angle_from_json(self.limits, m_id, actual_pos[i]))
                
            deg_vals = [float("nan")] * N_SENSORS
            for s in range(N_SENSORS):
                if s in parsed:
                    deg_vals[s] = parsed[s]["deg"]
                    
            self.log_data.append({
                "t_rel": t_rel,
                "phase": phase,
                "commanded": cmd_pos,
                "actual": actual_pos,
                "cmd_angles": cmd_angles,
                "act_angles": act_angles,
                "deg": deg_vals
            })

# -------------------------------------------------------------------------
# Interp & Main
# -------------------------------------------------------------------------
def move_interpolated(hand, io_lock, start_pos, end_pos, steps=40, sleep_dt=0.01):
    for i in range(1, steps + 1):
        alpha = i / float(steps)
        cmd = [int(start_pos[j] * (1 - alpha) + end_pos[j] * alpha) for j in range(len(start_pos))]
        if hand is not None:
            with io_lock:
                hand.set_pos(cmd)
        time.sleep(sleep_dt)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hand", type=str, default="right", choices=["left", "right"])
    parser.add_argument("--json", type=str, default=DEFAULT_JSON_PATH)
    parser.add_argument("--out-dir", type=str, default=DEFAULT_OUT_DIR)
    parser.add_argument("--points-per-joint", type=int, default=5, help="Number of random points to generate per joint")
    parser.add_argument("--wait-time", type=float, default=5.0, help="Wait time (seconds) between sending each point")
    args = parser.parse_args()

    if not os.path.exists(args.json):
        print(f"ERROR: Cannot find physical bounds file: {args.json}")
        sys.exit(1)

    with open(args.json, "r") as f:
        limits = json.load(f)
        
    # Dynamically inject exactly the maximum commanded ticks from the JSON so OPEN_POS perfectly matches Ground Truth natively 
    for config in limits.values():
        m_id = config["motor_id"]
        arr_idx = m_id - 1
        OPEN_POS[arr_idx] = int(config["commanded_max_ticks"])

    if Hand is None:
        print("ERROR: ruka_hand package not found. Cannot control dynamixels.")
        sys.exit(1)

    print(f"Connecting to {args.hand} hand...")
    try:
        hand = Hand(args.hand)
    except Exception as e:
        print(f"Failed to connect: {e}")
        sys.exit(1)

    io_lock = threading.Lock()
    logger = RandomSequenceLogger(args.serial_port, args.baud, hand, io_lock, limits)
    logger.start()

    try:
        print("Initializing directly to OPEN Hand Base...")
        logger.set_command_state(OPEN_POS, "init_open")
        with io_lock:
            current_pos = hand.read_pos()
            if current_pos:
                current_pos = [int(p) if p is not None else OPEN_POS[i] for i, p in enumerate(current_pos[:N_MOTORS])]
            else:
                current_pos = OPEN_POS.copy()
                
        move_interpolated(hand, io_lock, current_pos, OPEN_POS, steps=50)
        time.sleep(2.0)
        
        hand_state = OPEN_POS.copy()
        
        # Sort JSON keys strictly numerically
        joint_keys = sorted(list(limits.keys()), key=lambda x: int(x))
        total_points = args.points_per_joint * len(joint_keys)

        print("\n" + "="*60)
        print("Executing Sequential Per-Joint Bounded Random Movements")
        print(f"({args.points_per_joint} points per joint, {args.wait_time}s wait, {total_points} total sequence moves)")
        print("="*60)
        
        iteration = 1
        for s_idx_str in joint_keys:
            config = limits[s_idx_str]
            
            s_idx = int(s_idx_str)
            m_id = config["motor_id"]
            name = config["name"]
            arr_idx = m_id - 1
            
            c_min = int(config["commanded_min_ticks"])
            c_max = int(config["commanded_max_ticks"])
            
            # Ensure safe bounds min/max mapping if inverted
            safe_min = min(c_min, c_max)
            safe_max = max(c_min, c_max)
            span = safe_max - safe_min
            min_travel = int(span * 0.15)  # Enforce it moves at least 15% of its span
            
            print(f"\n--- Testing Joint: {name} (Sensor {s_idx} | Motor {m_id}) ---")

            for point_idx in range(1, args.points_per_joint + 1):
                current_tick = hand_state[arr_idx]
                
                new_tick = current_tick
                if span > 10:
                    for _ in range(50):
                        cand = random.randint(safe_min, safe_max)
                        if abs(cand - current_tick) >= min_travel:
                            new_tick = cand
                            break
                else:
                    new_tick = random.randint(safe_min, safe_max)
                
                print(f"[{iteration}/{total_points}] {name} Position {point_idx}/{args.points_per_joint}")
                print(f"   -> Randomizing M{m_id} bounds [{safe_min}, {safe_max}] -> Generated: {new_tick}")
                
                target_state = hand_state.copy()
                target_state[arr_idx] = new_tick
                
                phase_name = f"random_{name.replace(' ', '_')}_{point_idx}_S{s_idx}"
                logger.set_command_state(target_state, phase_name)
                
                # Interpolate to new state
                move_interpolated(hand, io_lock, hand_state, target_state, steps=40)
                hand_state = target_state.copy()
                
                print(f"   -> Holding Position for {args.wait_time} seconds...")
                time.sleep(args.wait_time)
                iteration += 1

            # --- Restore joint to Base OPEN position before starting next joint ---
            print(f"   [!] Restoring {name} (M{m_id}) to OPEN baseline...")
            target_state = hand_state.copy()
            target_state[arr_idx] = OPEN_POS[arr_idx]
            
            phase_name = f"reset_open_M{m_id}"
            logger.set_command_state(target_state, phase_name)
            
            # Interpolate to reset state smoothly
            move_interpolated(hand, io_lock, hand_state, target_state, steps=30)
            hand_state = target_state.copy()
            
            # Short stabilization delay
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nInterrupted.")
        
    # Always cleanly return the hand to OPEN_POS securely, even if interrupted
    print("\nReturning all joints to safe OPEN Base Position...")
    try:
        logger.set_command_state(OPEN_POS, "final_open")
        move_interpolated(hand, io_lock, hand_state, OPEN_POS, steps=50)
        time.sleep(1.0)
    except Exception:
        pass

    finally:
        print("Stopping logger and generating final CSV log output...")
        logger.stop()
        with io_lock:
            hand.close()
            
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.out_dir) / f"{stamp}_{args.points_per_joint}pts_{args.wait_time}s_random_bounded.csv"
    
    # Identify which motors map directly to JSON sensors to avoid plotting ghosts
    used_m_ids = sorted([int(config["motor_id"]) for config in limits.values()])
    
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        header = ["t_rel", "phase"]
        header += [f"deg_{i}" for i in range(N_SENSORS)]
        header += [f"cmd_{i}" for i in range(N_MOTORS)]
        header += [f"act_{i}" for i in range(N_MOTORS)]
        header += [f"expected_ang_cmd_{m_id}" for m_id in used_m_ids]
        header += [f"expected_ang_act_{m_id}" for m_id in used_m_ids]
        w.writerow(header)
        
        for row in logger.log_data:
            line = [row["t_rel"], row["phase"]]
            line += [f"{d:.2f}" if np.isfinite(d) else "nan" for d in row["deg"]]
            line += row["commanded"]
            line += row["actual"]
            line += [f"{row['cmd_angles'][m_id - 1]:.2f}" if np.isfinite(row['cmd_angles'][m_id - 1]) else "nan" for m_id in used_m_ids]
            line += [f"{row['act_angles'][m_id - 1]:.2f}" if np.isfinite(row['act_angles'][m_id - 1]) else "nan" for m_id in used_m_ids]
            w.writerow(line)
            
    print(f"Data saved cleanly to: {csv_path}")

if __name__ == "__main__":
    main()
