<h1 align="center" style="font-size: 2.0em; font-weight: bold; margin-bottom: 0; border: none; border-bottom: none;">RUKA-v2: Tendon Driven Open-Source Dexterous Hand with Wrist and Abduction for Robot Learning</h1>

##### <p align="center"> [Xinqi Liu<sup>*2</sup>](https://www.linkedin.com/in/xinqi-liu-889256299/), [Ruoxi Hu<sup>*1</sup>](https://www.linkedin.com/in/sissi-h-013288321/), [Alejandro Ojeda Olarte<sup>1</sup>](https://aojedao.github.io/Portfolio/), [Zhuoran Chen<sup>2</sup>](https://www.linkedin.com/in/zhuoran-chen-90a18a251/), [Kenny Ma<sup>1</sup>](https://kmang0.github.io/KMAWeb/), [Charles Cheng Ji<sup>1</sup>](https://charlesji.com/), [Lerrel Pinto<sup>1</sup>](https://lerrelpinto.com), [Raunaq Bhirangi<sup>1</sup>](https://raunaqbhirangi.github.io/), [Irmak Guzey<sup>1</sup>](https://irmakguzey.github.io/)</p>
##### <p align="center"><sup>1</sup>New York University&emsp;<sup>2</sup>New York University Shanghai</p>
##### <p align="center"><sup>*</sup>Equal contribution</p>

<p align="center">
  <img src="assets/ruka.gif">
 </p>

#####
<div align="center">
    <a href="https://ruka-hand-v2.github.io/"><img src="https://img.shields.io/static/v1?label=Project%20Page&message=Website&color=blue"></a> &ensp;
    <a href="https://arxiv.org/abs/2603.26660"><img src="https://img.shields.io/static/v1?label=Paper&message=Arxiv&color=red"></a> &ensp; 
    <a href="https://ruka-2.gitbook.io/ruka-v2"><img src="https://img.shields.io/static/v1?label=Hardware&message=Instructions&color=pink"></a> &ensp;
    
</div>

#####

## Installation
Download the repo, create the conda environment, install requirements and install the `ruka_hand` package.
```
git clone --recurse-submodules https://github.com/ruka-hand-v2/RUKA-v2
cd RUKA-v2
conda env create -f environment.yml
conda activate ruka_hand
pip install -r requirements.txt
pip install -e .
```


### Connecting RUKA-v2
Connect the RUKA-v2 hand to your workstation using a USB cable. Then, identify which port the USB is connected to. You can do this by plugging and unplugging the hand while observing the changes in the output of `ls /dev/ttyUSB*`. The port that appears when you plug in the hand corresponds to that hand (left or right).
Update the `USB_PORTS` dictionary in `ruka_hand/utils/constants.py` accordingly, e.g., `USB_PORTS = {"left": "/dev/ttyUSB0", "right": "/dev/ttyUSB1"}`.

You can try moving the motors to the reset position by running:
```
python scripts/reset_motors.py --hand_type <right|left>
```
If this moves the intended hand then it means that the initial software is completed!

**NOTE:**
If you get an error saying that 
```
serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0'
```
You might need to run following commands to add your user to `dialout` group and reboot the machine for it to take effect.
```
sudo usermod -aG dialout $USER
sudo reboot
```

### Calibrating Motor Ranges
Since RUKA-v2 is a tendon-driven hand, cable tensions can vary between different builds. To ensure consistency across builds, we provide a calibration script that determines the motor ranges by finding the fully curled finger bound and the in-tension bound (finger is fully open and the tendon is in tension).
Run the following command to save the motor limits to `RUKA/motor_limits/<left|right>_<tension|curl>_limits.npy`:  
```
python calibrate_motors.py --hand-type <left|right>
```
These motor limits are later used in `ruka_hand/control/hand.py`.

During calibration, you will be prompted to move the joints individually using the ↑/↓ keys to increase or decrease the motor commands, and to press enter when a desired positon is reached. Make sure to identify which motor is moved and ensure that the string is tensioned when moving to the open position. 

When moving joints, we sometimes observe that the knuckle joints don't fully curl. If you notice this behavior (for example, the index finger not fully curling despite changing the motor command, please gently push the finger to complete the curl.

After running the calibration, execute `python scripts/reset_motors.py --hand-type <right|left>`. This should move the fingers to a fully open position, with the tendons tensioned but the fingers remaining extended.


## Teleoperation

We provide scripts to teleoperate RUKA-v2 using Mediapipe
 ```
  python teleop_mediapipe.py
  ```
And we provide controllers in `ruka_hand/control/controller_retarget.py` that can be used to teleoperate RUKA-v2 using the [Oculus headset](https://www.meta.com/quest/quest-3/). 

## Franka-Teach
We provide code for teleoperating the RUKA-v2 mounted on a 7-DOF Franka Arm with single arm and bimanual setups using Open-Teach in the submodules `franka-teach-single` and `franka-teach-bimanual`, please refer to documentation in the [Franka-Teach](https://github.com/pianapolataa/Franka-Teach/blob/main/README.md) submodules.

## BAKU
We provide code for training your own visual BC policy with RUKA-v2 using BAKU in the submodule `BAKU`. Please refer to documentation in the [BAKU](https://github.com/pianapolataa/BAKU/blob/main/Instructions.md) submodule.

## Attachable Encoders
For the attachable encoders setup and data collection, please see the [Attachable Encoders README](ruka_encoders/README.md).

**Quick Start Commands:**
```bash
python calibration/recalibrate_limits.py --serial-port /dev/ttyACM0 --hand right
python calibration/extract_filtered_limits.py
python data_collection/random_joint_generator.py --points-per-joint 15 --wait-time 1.5
```
