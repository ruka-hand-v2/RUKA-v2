# RUKA Encoders (v2)

This directory contains the firmware, calibration tools, and data collection scripts for the magnetic angle sensors (AS5600) used on the RUKA hand. These tools are designed to map raw sensor readings to physical motor positions and analyze tracking performance.

---

## 📂 Directory Structure

### 1. [firmware/](firmware/)
Contains the code that runs on the microcontroller side to read the AS5600 sensors and stream data to a host PC.
- **`Microcontroller_AS5600.ino`**: Arduino/ESP32 firmware that reads multiple sensors via I2C using a TCA9548A multiplexer and outputs degree values over Serial.

### 2. [calibration/](calibration/)
Contains tools for defining the relationship between motor ticks and sensor degrees.
- **`recalibrate_limits.py`**: An interactive script that moves each motor to its limits and prompts for manual joint movement to capture absolute physical boundaries.
- **`extract_filtered_paper_limits.py`**: Processes calibration logs to generate a statistical "Ground Truth" mapping in a json format.
- **`manually_filtered_paper_limits.json`**: The reference configuration file containing the finalized motor-to-sensor mappings and limit bounds.

### 3. [data_collection/](data_collection/)
Contains automated scripts for testing the sensors and capturing synchronized motor/sensor data.
- **`random_joint_generator.py`**: Executes a sequence of random movements for each joint within the safe bounds defined in the calibration JSON and creates a csv file with the motor and sensor data.

---

## ⚙️ Workflow & Usage

Follow these steps to set up and use the encoder system:

### Step 1: Flash the Firmware
Upload the [Microcontroller_AS5600.ino](firmware/Microcontroller_AS5600.ino) to your microcontroller. Ensure all sensors are connected via the I2C multiplexer as expected by the code.

### Step 2: Capture Physical Limits
Run the interactive calibration script to capture the range of motion for each joint:
```bash
python calibration/recalibrate_limits.py --serial-port /dev/ttyACM0 --hand right
```
*Note: This will save CSV logs and a preliminary JSON to the `recalibrate_limits_logs/` folder.*

### Step 3: Finalize the Mapping
Run the extractor on your calibration logs to generate a noise-filtered JSON reference:
```bash
python calibration/extract_filtered_paper_limits.py
```
*Note: This will default to reading logs from calibration/recalibrate_limits_logs/ and outputting to calibration/manually_filtered_paper_limits.json.*

### Step 4: Run Automated Tests
Use the filtered JSON to run a bounded random movement sequence and log the tracking performance:
```bash
python data_collection/random_joint_generator.py --points-per-joint 15 --wait-time 1.5
```
*Note: This will default to using the JSON in calibration/ and saving new logs to data_collection/random_generator_logs/.*

---

## 🛠 Hardware Dependencies
- **Sensors**: AS5600 Magnetic Encoders and holders.
- **Robot**: RUKA v2 Hand.
- **Microcontroller**: ESP32 or Arduino ESP32.
- **I2C Multiplexer**: TCA9548A with Qwiic connectors.
- **Communication**: USB-C cable.

For hardware maintenance or sensor replacement instructions, refer to the [Disassembly.md](Disassembly.md) guide.
