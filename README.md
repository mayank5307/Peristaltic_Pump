# Smart Peristaltic Pump Controller (Arduino-Based)

An industrial-grade, precision-controlled peristaltic liquid dosing system powered by Arduino. Designed for automated liquid transfer, pharmaceutical dispensing, and laboratory batching applications.

## 📌 Features
* **Multi-Mode Operation:** 
  * **Dose Mode:** Dispenses precise volumetric quantities (mL).
  * **Pump Mode:** Runs continuous pumping cycles based on duration.
  * **Continuous Filling (Cont):** Automated bottle-filling automation with configurable dosing time, pause/rest time, and bottle counts.
  * **Calibration Mode (Cal.):** Fine-tunes micro-step pulses to exact liquid volume measurements.
* **On-Board UI Control:** 16x2 Parallel LCD with an analog joystick interface featuring dynamic multi-speed parameter acceleration.
* **Non-Volatile Memory (EEPROM):** Auto-saves user settings to maintain parameters across power cycles.
* **Serial / USB Interface:** Full remote operation and calibration via standardized serial command strings.
* **Stepper Motor Control:** Micro-stepping pulse generation with configurable rotation direction (CW/CCW) and dynamic delay calculation.

## 🛠️ Hardware Requirements
* **Microcontroller:** Arduino Uno / Mega / Nano
* **Display:** 16x2 HD44780 LCD
* **Actuator:** Stepper Motor (200 steps/rev standard) driven by a stepper motor driver board
* **Pump Head:** Peristaltic Pump Assembly (e.g., YZ1515x pump head)
* **Input:** 2-Axis Analog Joystick with push-button switch
* **Power Supply & Cooling:** High-current DC power setup with active cooling fan
