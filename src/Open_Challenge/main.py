"""
main.py

Main execution file for autonomous robot car.
Initializes hardware and camera, sets up ROIs, runs real-time control loop:
- Detects stripes and walls using color masks and ROIs
- Follows walls using PID control
- Handles turns and stripe counting
- Stops after completing course

Imports:
- functions.py: hardware and image processing utilities
- masks.py: LAB color mask definitions
"""

import sys
sys.path.append('/home/WRO/Desktop/')
import cv2
import RPi.GPIO as GPIO
import numpy as np
import time

from functions import *
from masks import rOrange, rBlack, rBlue

if __name__ == "__main__":
    time.sleep(1)  # allow system to stabilize

    # ---------------- GPIO BUTTON SETUP ----------------
    BUTTON_PIN = 26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # button active LOW

    print("Waiting for button press to start...")

    # Wait until button is pressed (LOW)
    while GPIO.input(BUTTON_PIN):  
        time.sleep(0.1)

    print("Button pressed! Starting bot...")

    # ---------------- USB CAMERA SETUP ----------------
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("Error: Could not open USB camera")
        exit()

    # ---------------- REGIONS OF INTEREST (ROIs) ----------------
    # ROIs define sub-areas in the camera frame for focused detection:
    # Each is [x1, y1, x2, y2] = (left, top, right, bottom)
    ROI1 = [0, 430, 384, 510]       # Bottom left, for left wall (black)
    ROI2 = [896, 430, 1280, 510]    # Bottom right, for right wall (black)
    ROI3 = [400, 360, 880, 460]     # Center stripe detection
    ROI4 = [0, 240, 160, 420]       # Far left (orange turn detection)
    ROI5 = [1120, 240, 1280, 420]   # Far right (blue turn detection)

    # ---------------- CONTROL VARIABLES ----------------
    lTurn = False      # Is a left turn in progress?
    rTurn = False      # Is a right turn in progress?

    # PID parameters for steering control
    kp = 0.15
    kd = 0.004
    straightConst = 110  # Center angle for straight driving

    angle = straightConst
    prevAngle = angle
    tDeviation = 25      # Angle deviation for sharp turns
    sharpRight = straightConst - tDeviation
    sharpLeft = straightConst + tDeviation
    maxRight = straightConst - 50
    maxLeft = straightConst + 50

    speed = 1500        # Motor speed (1000-2000 scale)

    aDiff = 0           # Area difference (right wall - left wall)
    prevDiff = 0

    debug = True        # Enable debug output and OpenCV window
    start = False       # Has the bot started moving?

    # --- COLOR DETECTION LOGIC ---
    activeColor = None      # "orange" or "blue" (course color)
    lastDetectTime = 0      # Last time a stripe was detected
    debounceSec = 5.0       # Stripe detection debounce time (seconds)

    colorCount = 0          # Number of stripes detected
    followEndTime = 0       # Time to stop after 12 stripes

    # ===================== MAIN LOOP =====================
    while True:
        ret, img = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        # Convert captured frame to LAB color space and blur for noise reduction
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)

        # --- WALL AND STRIPE DETECTION USING ROIs ---
        cListLeft = find_contours(img_lab, rBlack, ROI2)     # Right wall (bottom right ROI)
        cListRight = find_contours(img_lab, rBlack, ROI1)    # Left wall (bottom left ROI)
        cListOrange = find_contours(img_lab, rOrange, ROI3)  # Orange stripe (center ROI)
        cListBlue = find_contours(img_lab, rBlue, ROI3)      # Blue stripe (center ROI)

        leftArea = max_contour(cListLeft, ROI2)[0]   # Area of detected right wall
        rightArea = max_contour(cListRight, ROI1)[0] # Area of detected left wall

        currentTime = time.time()

        # ---------------- ACTIVE COLOR SELECTION ----------------
        # First detected stripe sets the course color (used for turns and counting)
        if activeColor is None:
            if len(cListOrange) > 0:
                activeColor = "orange"
                colorCount = 1
                lastDetectTime = currentTime
                print("Active color set to ORANGE (count=1)")
            elif len(cListBlue) > 0:
                activeColor = "blue"
                colorCount = 1
                lastDetectTime = currentTime
                print("Active color set to BLUE (count=1)")

        # ---------------- STRIPE COUNTING WITH DEBOUNCE ----------------
        # Stripe is counted only if debounce time has passed since last detection
        if activeColor == "orange" and len(cListOrange) > 0 and (currentTime - lastDetectTime) > debounceSec:
            colorCount += 1
            lastDetectTime = currentTime
            print(f"Orange stripe count: {colorCount}")

        elif activeColor == "blue" and len(cListBlue) > 0 and (currentTime - lastDetectTime) > debounceSec:
            colorCount += 1
            lastDetectTime = currentTime
            print(f"Blue stripe count: {colorCount}")

        # ---------------- STOP AFTER 12 STRIPES ----------------
        # When 12 stripes detected, stop after 4 seconds
        if colorCount == 12 and followEndTime == 0:
            followEndTime = currentTime + 7
            print("12 stripes detected! Stopping after 4 seconds...")

        if followEndTime > 0 and currentTime >= followEndTime:
            print("Bot stopped after 12 counts + 4s")
            stop_car()
            set_servo_angle(straightConst)
            break

        # ---------------- TURN HANDLING BASED ON COLOR AND WALL PRESENCE ----------------
        # For blue course: If left ROI (ROI4) loses wall, rotate left until left wall reacquired
        if activeColor == "blue":
            blackROI4 = max_contour(find_contours(img_lab, rBlack, ROI4), ROI4)[0]
            if blackROI4 <= 50:
                print("Blue turn: rotating LEFT until ROI1 sees black")
                startTime = time.time()
                while True:
                    write(70)  # steer hard left
                    ret, img = cap.read()
                    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
                    leftCheck = max_contour(find_contours(img_lab, rBlack, ROI1), ROI1)[0]
                    if leftCheck > 200 or (time.time() - startTime) > 5:
                        print("ROI1 black detected or timeout, resuming PID")
                        break

        # For orange course: If right ROI (ROI5) loses wall, rotate right until right wall reacquired
        elif activeColor == "orange":
            blackROI5 = max_contour(find_contours(img_lab, rBlack, ROI5), ROI5)[0]
            if blackROI5 <= 50:
                print("Orange turn: rotating RIGHT until ROI2 sees black")
                startTime = time.time()
                while True:
                    write(150)  # steer hard right
                    ret, img = cap.read()
                    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
                    rightCheck = max_contour(find_contours(img_lab, rBlack, ROI2), ROI2)[0]
                    if rightCheck > 200 or (time.time() - startTime) > 5:
                        print("ROI2 black detected or timeout, resuming PID")
                        break

        # ---------------- PID WALL FOLLOWING (STEERING CONTROL) ----------------
        # Calculate area difference (right wall - left wall)
        # Positive aDiff: steer left (more right wall detected)
        # Negative aDiff: steer right (more left wall detected)
        aDiff = rightArea - leftArea
        angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))

        # If wall is lost (low area), initiate sharp turn correction
        if leftArea <= 150 and not rTurn:
            lTurn = True
        elif rightArea <= 150 and not lTurn:
            rTurn = True

        # ---------------- INITIAL STARTUP ----------------
        if not start:
            multi_write([angle, speed])  # set steering and speed
            start = True

        # ---------------- STEERING CORRECTION LOGIC ----------------
        if angle != prevAngle:
            if lTurn or rTurn:
                # End sharp turn when wall reacquired
                if (rightArea > 1500 and rTurn) or (leftArea > 1500 and lTurn):
                    lTurn = False
                    rTurn = False
                    prevDiff = 0
                elif lTurn:
                    angle = min(max(angle, sharpLeft), maxLeft)
                elif rTurn:
                    angle = max(min(angle, sharpRight), maxRight)
                write(angle)
                time.sleep(0.01)
            else:
                write(max(min(angle, sharpLeft), sharpRight))
                time.sleep(0.01)

        prevDiff = aDiff
        prevAngle = angle

        # ---------------- DEBUG DISPLAY ----------------
        if debug:
            if cv2.waitKey(1) == ord('q'):
                stop_car()
                break
            img = display_roi(img, [ROI1, ROI2, ROI3, ROI4, ROI5], (255, 204, 0))
            cv2.imshow("finalColor", img)
            variables = {
                "left wall area": leftArea,
                "right wall area": rightArea,
                "Active color": activeColor,
                "Color count": colorCount 
            }
            display_variables(variables)

    # ---------------- CLEANUP ON EXIT ----------------
    cap.release()
    cv2.destroyAllWindows()
    stop_car()
    set_servo_angle(straightConst)