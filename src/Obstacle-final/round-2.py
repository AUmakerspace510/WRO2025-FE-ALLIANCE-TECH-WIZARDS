import sys
sys.path.append('/home/WRO/Desktop/')
import cv2
import RPi.GPIO as GPIO
import numpy as np
import time

from functions import *
from masks import rOrange, rBlack, rBlue, rRed, rGreen  # added red & green

if _name_ == "_main_":
    time.sleep(1)  # allow system to stabilize

    # ---------------- GPIO BUTTON SETUP ----------------
    BUTTON_PIN = 26
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # button active LOW

    print("Waiting for button press to start...")

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

    # ---------------- ROIs ----------------
    ROI1 = [0, 430, 384, 510]       # bottom left
    ROI2 = [896, 430, 1280, 510]    # bottom right
    ROI3 = [400, 360, 880, 460]     # center stripe detection
    ROI4 = [0, 240, 160, 420]       # left (orange turn condition)
    ROI5 = [1120, 240, 1280, 420]   # right (blue turn condition)
    ROI6 = [262, 410, 1018, 460]

    # ---------------- CONTROL VARIABLES ----------------
    lTurn, rTurn = False, False

    kp = 0.15
    kd = 0.004
    straightConst = 110

    angle = straightConst
    prevAngle = angle
    tDeviation = 25
    sharpRight = straightConst - tDeviation
    sharpLeft = straightConst + tDeviation
    maxRight = straightConst - 50
    maxLeft = straightConst + 50

    speed = 1600   # reduced speed for smoother motion

    aDiff, prevDiff = 0, 0

    debug = True
    start = False

    # --- COLOR DETECTION LOGIC ---
    activeColor = None
    lastDetectTime = 0
    debounceSec = 2.0

    colorCount = 0
    followAfter12 = False
    followEndTime = 0

    # ---------------- MAIN LOOP ----------------
    while True:
        ret, img = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0)

        # ---------- FIND CONTOURS ----------
        cListLeft = find_contours(img_lab, rBlack, ROI2)
        cListRight = find_contours(img_lab, rBlack, ROI1)
        cListOrange = find_contours(img_lab, rOrange, ROI3)
        cListBlue = find_contours(img_lab, rBlue, ROI3)

        leftArea = max_contour(cListLeft, ROI2)[0]
        rightArea = max_contour(cListRight, ROI1)[0]

        # ---------- OBSTACLE DETECTION ----------
        redCnt = find_contours(img_lab, rRed, ROI6)
        greenCnt = find_contours(img_lab, rGreen, ROI6)

        redArea, _, _, _ = max_contour(redCnt, ROI6)
        greenArea, _, _, _ = max_contour(greenCnt, ROI6)
        color_threshold = 100

        currentTime = time.time()

        # ---------- ACTIVE COLOR SELECTION ----------
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

        # ---------- COLOR COUNTING ----------
        if activeColor == "orange" and len(cListOrange) > 0 and (currentTime - lastDetectTime) > debounceSec:
            colorCount += 1
            lastDetectTime = currentTime
            print(f"Orange stripe count: {colorCount}")

        elif activeColor == "blue" and len(cListBlue) > 0 and (currentTime - lastDetectTime) > debounceSec:
            colorCount += 1
            lastDetectTime = currentTime
            print(f"Blue stripe count: {colorCount}")

        # ---------- STOP AFTER 12 COUNTS ----------
        if colorCount == 12 and followEndTime == 0:
            followEndTime = currentTime + 4
            print("12 stripes detected! Stopping after 4 seconds...")

        if followEndTime > 0 and currentTime >= followEndTime:
            print("Bot stopped after 12 counts + 4s")
            stop_car()
            set_servo_angle(straightConst)
            break

        # ---------- OBSTACLE AVOIDANCE ----------
        if redArea > color_threshold:
            print("RED obstacle detected â†’ steer RIGHT")
            set_servo_angle(160)
            multi_write([160, speed])
            continue  # skip PID when avoiding

        elif greenArea > color_threshold:
            print("GREEN obstacle detected â†’ steer LEFT")
            set_servo_angle(60)
            multi_write([60, speed])
            continue

        # ---------- TURN HANDLING ----------
        if activeColor == "blue":
            blackROI4 = max_contour(find_contours(img_lab, rBlack, ROI4), ROI4)[0]
            if blackROI4 <= 50:
                print("Blue turn: rotating LEFT until ROI1 sees black")
                startTime = time.time()
                while True:
                    write(70)
                    ret, img = cap.read()
                    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
                    leftCheck = max_contour(find_contours(img_lab, rBlack, ROI1), ROI1)[0]
                    if leftCheck > 50 or (time.time() - startTime) > 5:
                        print("ROI1 black detected or timeout, resuming PID")
                        set_servo_angle(straightConst)
                        break

        elif activeColor == "orange":
            blackROI5 = max_contour(find_contours(img_lab, rBlack, ROI5), ROI5)[0]
            if blackROI5 <= 50:
                print("Orange turn: rotating RIGHT until ROI2 sees black")
                startTime = time.time()
                while True:
                    write(150)
                    ret, img = cap.read()
                    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
                    rightCheck = max_contour(find_contours(img_lab, rBlack, ROI2), ROI2)[0]
                    if rightCheck > 50 or (time.time() - startTime) > 5:
                        print("ROI2 black detected or timeout, resuming PID")
                        set_servo_angle(straightConst)
                        break

        # ---------- PID CONTROL ----------
        aDiff = rightArea - leftArea
        angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))

        if leftArea <= 150 and not rTurn:
            lTurn = True
        elif rightArea <= 150 and not lTurn:
            rTurn = True

        if not start:
            multi_write([angle, speed])
            start = True

        if angle != prevAngle:
            if lTurn or rTurn:
                if (rightArea > 1500 and rTurn) or (leftArea > 1500 and lTurn):
                    lTurn, rTurn = False, False
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

        # ---------- DEBUG DISPLAY ----------
        if debug:
            if cv2.waitKey(1) == ord('q'):
                stop_car()
                break
            img = display_roi(img, [ROI1, ROI2, ROI3, ROI4, ROI5, ROI6], (255, 204, 0))
            cv2.imshow("finalColor", img)
            variables = {
                "left wall area": leftArea,
                "right wall area": rightArea,
                "Active color": activeColor,
                "Color count": colorCount,
                "Red area": redArea,
                "Green area": greenArea
            }
            display_variables(variables)

    # ---------------- CLEANUP ----------------
    cap.release()
    cv2.destroyAllWindows()
    stop_car()
    set_servo_angle(straightConst)