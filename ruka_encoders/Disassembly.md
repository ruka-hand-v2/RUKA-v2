# RUKA Hand Sensor Disassembly Guide

This document outlines the step-by-step procedure for safely disassembling the magnetic encoders and sensor holders from the RUKA hand.

---

## 1. Initial Preparation
Before touching the mechanical components, clear the electronics and cabling:
- **Disconnect Qwiic Cables**: Carefully unplug the Qwiic cables from every sensor.
- **Remove Multiplexer**: Unscrew and remove the red I2C multiplexer board from the back of the hand.

---

## 2. Index Finger Sensors (DIP, PIP, MCP)
The assembly for the Index finger is interconnected by tension and pins. Follow this order:

1. **Relieve Tension**: Remove the screws holding the springs for the **DIP** and **PIP** joints.
2. **Loosen MCP**: Remove the outer-most screw on the MCP section of the index finger.
3. **DIP Extraction**:
   - Push the DIP sensor slightly backward.
   - Remove the small circular holder on the pin (this secures the magnet).
   - Extract the 3D-printed part, then slide the sensor holder out. 
   - *Note: If the pin prevents the holder from sliding, use **pliers** to remove the pin first.*
4. **MCP Extraction**:
   - Reach the magnet in the next pin. If it is difficult to grasp, use **pliers**.
   - Loosen the magnet from the pin (it will likely attach itself to the sensor once free).
   - Slide the MCP sensor holder outward. 
   - *Note: This part is tight and may require significant force to slide off.*
5. **Reassembly of Frame**: Once holders are removed, replace the springs, screws, and pins to maintain the finger's structural integrity.

---

## 3. Index Abduction Sensor
The abduction sensor is a self-contained unit located near the base of the index finger:

1. **Unscrew**: Locate the single screw visible from the top of the hand and remove it.
2. **Slide**: Slide the holder out of its slot. 
3. **Magnet Cap**: A small magnet holder/cap may fall out during this step. Replace this cap (with the magnet inside) and re-fasten the screw after the main sensor holder has been removed.

---

## 4. Thumb Sensors
The thumb sensors are removed starting from the tip and moving toward the wrist:

### Thumb DIP
- Follow the same process as the index DIP: Unscrew the tensioners, slide the holder off the joint, and release the sensor.

### Thumb MCP
- This joint is easier to manage: Simply remove the **nut** located at the bottom of the holder.
- Slide the holder directly off the assembly.

### Thumb CMC
- The CMC holder is secured at two points on the bottom of the wrist.
- **Inner Screw**: This can be reached with a standard **screwdriver**.
- **Outer Screw**: Due to the angle, you may need an **Allen key** to reach this point.
- Release both screws, slide the holder down toward the arm, and replace the screws into the frame for safekeeping.