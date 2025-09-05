# WRO2025 FE ALLIANCE TECH WIZARDS - Autonomous Raspberry Pi Robot

This repository contains three core Python files that together enable a Raspberry Pi-powered robot to autonomously follow colored stripes, detect walls, make turns, and stop after completing its course. Below you’ll find detailed documentation for each file.

---

## 1. `masks.py`

### Purpose
Defines color masks for use in image processing. These are LAB color space ranges which help the robot identify different colored stripes (magenta, red, green, blue, orange) and walls (black) in the environment.

### How It Works
- Each mask is a list with two NumPy arrays: the lower and upper bounds for LAB color segmentation.
- These masks are imported in other files and used to isolate specific colors in images captured by the camera.

### Example Usage
Used in the `find_contours` function in `functions.py` and the main loop in `main.py` to detect colored stripes and walls.

### File Reference
```python
import numpy as np

rMagenta = [np.array([22, 150, 50], np.uint8), np.array([200, 200, 150], np.uint8)]
rRed     = [np.array([0, 150, 120], np.uint8), np.array([255, 220, 180], np.uint8)]
rGreen   = [np.array([0, 50, 0], np.uint8), np.array([255, 120, 255], np.uint8)]
rBlue    = [np.array([0, 120, 60], np.uint8), np.array([255, 160, 120], np.uint8)]
rOrange  = [np.array([0, 117, 150], np.uint8), np.array([255, 200, 200], np.uint8)]
rBlack   = [np.array([0, 105, 105], np.uint8), np.array([80, 151, 151], np.uint8)]

lotType = "light"  # 0 for dark purple, 1 for magenta (not used in main code)
```

---

## 2. `functions.py`

### Purpose
Contains all hardware control and image processing functions. This file abstracts GPIO operations and OpenCV image processing into reusable functions.

### Key Functions

- **GPIO Setup:** Configures motor and servo pins, establishes PWM channels.
- **set_motor(speed):** Controls direction and speed of DC motor using L298N.
- **set_servo_angle(angle):** Sets steering angle for the servo motor.
- **write(value):** Unified interface for controlling either steering angle or motor speed.
- **multi_write(sequence):** Executes a list of actions (angles, speeds, delays).
- **stop_car():** Safely stops the robot and resets steering.
- **move_backward_with_mirror(forward_angle, duration, speed):** Reverses with mirrored steering (for advanced maneuvers).
- **display_roi(img, ROIs, color):** Draws rectangles for specified regions of interest (ROIs) on the image, useful for debugging.
- **find_contours(img_lab, lab_range, ROI):** Segments a region of the image for a specific color and finds contours.
- **max_contour(contours, ROI):** Returns area and position of the largest contour found in an ROI.
- **display_variables(variables):** Prints debug information about main variables (e.g., detected wall areas).

### How It Works
- The robot’s hardware (motors/servo) are controlled indirectly by calling these functions from the main loop.
- Image processing functions enable stripe/wall detection, which informs the robot's navigation logic.

### Example Usage
Used by `main.py` to execute movement, steering, and image analysis every frame.

### File Reference
```python
# Example: set_motor(50) moves forward, set_servo_angle(110) steers straight.
# Example: find_contours(img_lab, rBlack, ROI1) gets contours for black walls in left ROI.
```

---

## 3. `main.py`

### Purpose
The main execution file. This script launches the robot’s operational loop: waiting for button press, initializing camera and hardware, detecting stripes and walls, handling turns, counting stripes, and stopping after finishing the course.

### How It Works

1. **Startup:**
   - Waits for the physical button press to start.
   - Initializes camera and sets capture parameters.

2. **ROIs (Regions of Interest):**
   - Defines five ROIs for focused color and wall detection:
     - ROI1: Bottom left (left wall)
     - ROI2: Bottom right (right wall)
     - ROI3: Center (stripe detection)
     - ROI4: Far left (orange turn trigger)
     - ROI5: Far right (blue turn trigger)

3. **Main Loop:**
   - Captures frame, processes in LAB color space.
   - Detects contours for walls and stripes using masks from `masks.py` and functions from `functions.py`.
   - Uses the area of detected contours to maintain wall-following via PID steering logic.
   - Counts stripes (with debounce) and stops after 12 stripes.
   - Handles turns when wall is lost in ROI4/ROI5 (depending on detected color).
   - Displays debug info and stops on user command or course completion.

### Example Usage
Run this file to start the robot after wiring and camera setup.

### File Reference
```python
from functions import *
from masks import rOrange, rBlack, rBlue

# Main loop:
# while True:
#     ret, img = cap.read()
#     img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
#     cListLeft = find_contours(img_lab, rBlack, ROI2)
#     ... (see full file for details)
```

---

## Additional Notes

- **Dependencies:** Make sure to install `numpy`, `RPi.GPIO`, and `opencv-python` on your Raspberry Pi.
- **Hardware:** Connect L298N motor driver and a servo as per pin definitions in `functions.py`.
- **Camera:** Any USB webcam compatible with OpenCV should work.
- **Debugging:** Enable `debug = True` in `main.py` to visualize ROIs and variables via OpenCV window.
- **Safety:** The `stop_car()` function is called on exit or after finishing the course to ensure the robot halts safely.

---

## Quick Start

1. Wire up your robot according to the pin definitions in `functions.py`.
2. Place all three files (`masks.py`, `functions.py`, `main.py`) in the same directory.
3. Run `main.py` on your Raspberry Pi.
4. Press the button to start the robot. Watch it follow colored stripes and walls, count stripes, make turns, and stop after the course!

---