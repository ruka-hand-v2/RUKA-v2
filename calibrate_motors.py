import os
import sys
import termios
import tty
import numpy as np
from ruka_hand.control.hand import Hand
from ruka_hand.utils.file_ops import get_repo_root

def get_key():
    """Capture Up, Down, Enter, or Ctrl+C."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Ctrl+C
        if ch == '\x03':
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("\n\n[EXIT] Keyboard Interrupt.")
            sys.exit(0)
        # Arrow keys start with ESC [
        if ch == "\x1b":
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class HandCalibrator:
    def __init__(self, data_save_dir, hand_type):
        self.hand = Hand(hand_type)
        self.num_motors = 16
        self.tendon_motor_ids = list(range(1, 17)) 
        
        self.curled_path = os.path.join(data_save_dir, f"{hand_type}_curl_limits.npy")
        self.tension_path = os.path.join(data_save_dir, f"{hand_type}_tension_limits.npy")

        self.final_curled = np.zeros(16, dtype=int)
        self.final_tensioned = np.zeros(16, dtype=int)

    def crawl_motor(self, motor_id, label, step=50):
        print(f"--- Motor {motor_id}: {label} ---")
        print("↑: Increase | ↓: Decrease | ENTER: Save")
        
        idx = motor_id - 1
        pos = np.array(self.hand.read_pos())
        
        while True:
            actual_cur = self.hand.read_single_cur(motor_id)
            actual_pos = self.hand.read_pos()[idx]
            
            sys.stdout.write(f"\rTarget: {pos[idx]:4} | Real: {actual_pos:4} | Cur: {actual_cur:5} ")
            sys.stdout.flush()
            
            k = get_key()
            
            if k in ("\r", "\n"):
                print(f"\n[OK] Saved: {actual_pos}\n")
                return int(actual_pos)
            elif k == "\x1b[A": # Up Arrow
                pos[idx] = max(min(pos[idx] + step, 4095), 0)
                self.hand.set_pos(pos)
            elif k == "\x1b[B": # Down Arrow
                pos[idx] = max(min(pos[idx] - step, 4095), 0)
                self.hand.set_pos(pos)

    def calibrate(self):
        for mid in self.tendon_motor_ids:
            print(f"================ MOTOR {mid} ================")
            self.final_curled[mid-1] = self.crawl_motor(mid, "CURLED/CLOSED")
            self.final_tensioned[mid-1] = self.crawl_motor(mid, "TENSIONED/OPEN")
            
            # Relax motor to open position
            pos = np.array(self.hand.read_pos())
            pos[mid-1] = self.final_tensioned[mid-1]
            self.hand.set_pos(pos)

        
        np.save(self.curled_path, self.final_curled)
        np.save(self.tension_path, self.final_tensioned)
        print(f"\nSaved 16 joints to:\n{self.curled_path}\n{self.tension_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-ht", "--hand-type", type=str, default="right")
    args = parser.parse_args()

    save_dir = os.path.join(get_repo_root(), "curl_limits")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    calibrator = HandCalibrator(save_dir, args.hand_type)
    calibrator.calibrate()