# This script will move the the fingers with order automatically
import os
import sys
import time
import termios
import tty

import numpy as np

from ruka_hand.control.hand import *
from ruka_hand.utils.file_ops import get_repo_root


def get_key():
    """Capture a single key press (including arrow escape sequences)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            # likely an escape sequence; read the next two chars
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class HandCalibrator:
    def __init__(
        self,
        data_save_dir,
        hand_type,
        curr_lim=50,
        testing=False,
        motor_ids=[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
    ):
        self.hand = Hand(hand_type)
        self.curr_lim = curr_lim
        self.testing = testing
        self.motor_ids = motor_ids
        self.data_save_dir = data_save_dir  # directory only

        # Save paths
        self.curled_path = os.path.join(
            self.data_save_dir, f"{hand_type}_curl_limits.npy"
        )
        self.tension_path = os.path.join(
            self.data_save_dir, f"{hand_type}_tension_limits.npy"
        )

    def find_bound(self, motor_id):
        # Add cases for specific motors here
        t = 2
        if motor_id in [4, 5]:
            if motor_id == 4:
                self.curr_lim = 250
            else:
                self.curr_lim = 200
            t = 5
        if self.testing:
            print("------------ MOTOR ", motor_id, " ------------")
        if self.hand.hand_type == "right":
            start_pos = 100
            f = 1
        elif self.hand.hand_type == "left":
            start_pos = 4000
            f = -1
        l_bound = 100
        u_bound = 4000
        pos = np.array([start_pos] * 11)
        cur = 1000000
        while abs(u_bound - l_bound) > 10 or f * cur > self.curr_lim:
            com_pos = (u_bound + l_bound) // 2 - 1
            pos[motor_id - 1] = com_pos
            self.hand.set_pos(pos)
            time.sleep(t)
            cur = self.hand.read_single_cur(motor_id)
            pres_pos = self.hand.read_pos()[motor_id - 1]
            if self.testing:
                print(u_bound, "    ", l_bound, "         ", com_pos, "         ", pres_pos)
                print(cur)
            if f * cur < self.curr_lim:
                if self.hand.hand_type == "right":
                    l_bound = pres_pos + 1
                    u_bound -= 1
                else:
                    u_bound = pres_pos - 1
                    l_bound += 1
            else:
                if self.hand.hand_type == "right":
                    u_bound = pres_pos + 1
                else:
                    l_bound = pres_pos - 1
        return pres_pos

    def find_curled(self):
        curled = np.zeros(len(self.motor_ids), dtype=int)
        for i, mid in enumerate(self.motor_ids):
            curled[i] = int(self.find_bound(mid))
        return curled

    def estimate_tensioned_from_curled(self, curled):
        """Approximate tensioned bounds first (previous method)."""
        # Keep your existing fixed-range estimate (1100) and respect left/right direction
        f = 1 if self.hand.hand_type == "right" else -1
        return np.array([int(x - f * 1100) for x in curled], dtype=int)

    def interactive_refine_tensioned(self, tensioned_init, step=10):
        """
        Iterate through motors; arrows adjust ±step; Enter confirms & advances.
        Up/Right arrows: +step, Down/Left arrows: -step relative to servo direction.
        """
        # Ensure we can set positions for all 11 motors
        current_pos = np.array(self.hand.read_pos())

        # Start each motor at its initial tensioned guess
        tensioned = tensioned_init.copy()

        # Direction factor: moves in the "open/tension" direction
        f = 1 if self.hand.hand_type == "right" else -1

        print("\n--- Tensioned Calibration ---")
        print("Use ↑/→ to increase, ↓/← to decrease (±10). Press Enter to confirm motor.\n")

        for mid in self.motor_ids:
            idx = mid - 1
            # Move all motors to safe neutral, then set current motor to its guess
            pos = current_pos.copy()
            pos[idx] = tensioned[idx]
            self.hand.set_pos(pos)
            time.sleep(0.2)

            while True:
                print(f"[Motor {mid}] Current candidate: {pos[idx]}")
                print("Adjust with arrows, Enter to save, 'q' to abort this motor.")

                k = get_key()

                if k in ("\r", "\n"):  # Enter confirms
                    tensioned[idx] = int(pos[idx])
                    print(f"Saved Motor {mid} tensioned = {tensioned[idx]}\n")
                    break
                elif k in ("\x1b[A", "\x1b[C"):  # Up or Right arrow
                    pos[idx] = max(min(pos[idx] + step * f, 4090), 10)
                    self.hand.set_pos(pos)
                elif k in ("\x1b[B", "\x1b[D"):  # Down or Left arrow
                    pos[idx] = max(min(pos[idx] - step * f, 4090), 0)
                    self.hand.set_pos(pos)
                elif k.lower() == "q":
                    print(f"Aborted adjustments for Motor {mid}; keeping {tensioned[idx]}\n")
                    break
                else:
                    # ignore other keys
                    pass

        print("Final tensioned array:\n", tensioned)
        return tensioned.astype(int)

    def save_curled_limits(self):
        curled = self.find_curled()
        np.save(self.curled_path, curled)
        print(f"Saved curled limits to {self.curled_path}")

    def save_tensioned_limits(self):
        """
        Runs the interactive tension pass.
        Requires curled to exist (either just measured or previously saved).
        """
        if os.path.exists(self.curled_path):
            curled = np.load(self.curled_path)
        else:
            print("Curled limits not found; running curled calibration first...")
            curled = self.find_curled()
            np.save(self.curled_path, curled)
            print(f"Saved curled limits to {self.curled_path}")

        t_init = self.estimate_tensioned_from_curled(curled)
        t_refined = self.interactive_refine_tensioned(t_init, step=10)
        np.save(self.tension_path, t_refined)
        print(f"Saved tensioned limits to {self.tension_path}")


def parse_args():
    import argparse

    parser = argparse.ArgumentParser(description="Calibrate RUKA hand motors")
    parser.add_argument(
        "-ht",
        "--hand-type",
        type=str,
        default="right",
        choices=["right", "left"],
        help="Type of hand to calibrate (right or left)",
    )
    parser.add_argument(
        "--testing",
        type=bool,
        default=True,
        help="Enable testing mode with debug prints",
    )
    parser.add_argument(
        "--curr-lim", type=int, default=50, help="Current limit for calibration"
    )
    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        choices=["curl", "tension", "both"],
        default="both",
        help="Which calibration(s) to run: curled only, tension only, or both (default).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    repo_root = get_repo_root()
    save_dir = f"{repo_root}/curl_limits"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    calibrator = HandCalibrator(
        data_save_dir=save_dir,
        hand_type=args.hand_type,
        curr_lim=args.curr_lim,
        testing=args.testing,
    )

    if args.mode in ("curl", "both"):
        calibrator.save_curled_limits()
    if args.mode in ("tension", "both"):
        calibrator.save_tensioned_limits()
