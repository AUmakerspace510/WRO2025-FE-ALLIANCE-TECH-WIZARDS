"""
functions.py

Hardware control and image processing utilities for Raspberry Pi robot car.
Controls DC motors and servo, provides unified write interface, and
includes image processing functions for contour detection and ROI visualization.
"""

import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# ================== GPIO SETUP ==================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor driver pins (L298N)
IN1, IN2, ENA = 17, 27, 19
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Motor speed control PWM
pwm_motor = GPIO.PWM(ENA, 1000)  # 1kHz PWM for motor
pwm_motor.start(0)

# Servo pin for steering
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz servo PWM
servo_pwm.start(0)

# ================== MOTOR CONTROL ==================
def set_motor(speed):
    """
    Control DC motor speed and direction.
    Args:
        speed (int): -100 to 100. Positive = forward, negative = backward.
    Fires:
        Used for every movement or stop command.
    """
    if speed > 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif speed < 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(min(abs(speed), 100))

# ================== SERVO CONTROL ==================
CENTER_ANGLE = 110  # Center/straight steering
MIN_ANGLE = 60      # Full left
MAX_ANGLE = 160     # Full right

def set_servo_angle(angle):
    """
    Set servo angle for steering (clamped to allowed range).
    Args:
        angle (int): Desired angle (60-160).
    Fires:
        On every steering update.
    """
    angle = max(MIN_ANGLE, min(MAX_ANGLE, angle))
    duty = 2 + (angle / 18)  # Map angle to PWM duty cycle
    servo_pwm.ChangeDutyCycle(duty)
    print(f"[SERVO] Angle: {angle}, Duty: {duty}")  # Debug print
    time.sleep(0.02)  # Allow servo to reach position

# ================== HIGH-LEVEL FUNCTIONS ==================
def write(value):
    """
    Unified write: controls motor (1000-2000) or servo (60-160).
    Args:
        value (int): If 60-160, sets steering. If 1000-2000, sets motor speed.
    Fires:
        Called for all movement/steering commands.
    """
    if MIN_ANGLE <= value <= MAX_ANGLE:  # steering
        set_servo_angle(value)
    elif 1000 <= value <= 2000:  # motor PWM style (1000-2000)
        speed = int((value - 1000) / 10)  # map to 0-100
        set_motor(speed)

def multi_write(sequence):
    """
    Execute a sequence of actions (angles, speeds, sleeps).
    Args:
        sequence (list): List of values (steering, speed, sleep sec).
    Fires:
        Used for multi-step startup or maneuver.
    """
    for action in sequence:
        if isinstance(action, (int, float)):
            if action < 25:  # If <25, treat as sleep time (seconds)
                time.sleep(action)
            else:
                write(action)

def stop_car():
    """
    Stop all movement, center steering, close OpenCV windows.
    Fires:
        On exit or stop condition.
    """
    set_motor(0)
    set_servo_angle(CENTER_ANGLE)
    cv2.destroyAllWindows()

def move_backward_with_mirror(forward_angle, duration=1, speed=50):
    """
    Move backward with steering mirrored to last forward angle.
    Args:
        forward_angle (int): Previous steering angle.
        duration (float): Move time in seconds.
        speed (int): Backward speed.
    Fires:
        For reversing maneuvers.
    """
    mirror_angle = CENTER_ANGLE - (forward_angle - CENTER_ANGLE)
    mirror_angle = max(MIN_ANGLE, min(MAX_ANGLE, mirror_angle))
    print(f"[BACKWARD] Forward angle: {forward_angle}, mirrored: {mirror_angle}")
    set_servo_angle(mirror_angle)
    time.sleep(0.1)
    set_motor(-abs(speed))
    time.sleep(duration)
    set_motor(0)
    set_servo_angle(forward_angle)
    time.sleep(0.05)

# ================== IMAGE PROCESSING ==================
def display_roi(img, ROIs, color):
    """
    Draw rectangles for Regions of Interest (ROIs) on the image.
    Args:
        img (np.ndarray): Image to draw on.
        ROIs (list): Each ROI is [x1, y1, x2, y2].
        color (tuple): BGR color for lines.
    Purpose:
        Visual debug of detection zones.
    """
    for ROI in ROIs:
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[0], ROI[1]), (ROI[0], ROI[3]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[2], ROI[1]), color, 4)
        img = cv2.line(img, (ROI[2], ROI[3]), (ROI[0], ROI[3]), color, 4)
    return img

def find_contours(img_lab, lab_range, ROI):
    """
    Find contours of given LAB color range in a specified ROI.
    Args:
        img_lab (np.ndarray): LAB image.
        lab_range (list): [lower LAB, upper LAB].
        ROI (list): [x1, y1, x2, y2].
    Returns:
        List of contours found.
    Fires:
        Every frame, for color/wall detection.
    """
    img_segmented = img_lab[ROI[1]:ROI[3], ROI[0]:ROI[2]]
    lower_mask = np.array(lab_range[0])
    upper_mask = np.array(lab_range[1])
    mask = cv2.inRange(img_segmented, lower_mask, upper_mask)
    kernel = np.ones((5, 5), np.uint8)
    eMask = cv2.erode(mask, kernel, iterations=1)
    dMask = cv2.dilate(eMask, kernel, iterations=1)
    contours = cv2.findContours(dMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    return contours

def max_contour(contours, ROI):
    """
    Find largest contour in list, return area and adjusted coordinates.
    Args:
        contours (list): contour list.
        ROI (list): [x1, y1, x2, y2].
    Returns:
        [maxArea, maxX, maxY, mCnt]: Area, position, contour.
    Fires:
        Used for stripe/wall area calculation.
    """
    maxArea = 0
    maxY = 0
    maxX = 0
    mCnt = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            x += ROI[0] + w // 2
            y += ROI[1] + h
            if area > maxArea:
                maxArea = area
                maxY = y
                maxX = x
                mCnt = cnt
    return [maxArea, maxX, maxY, mCnt]

def display_variables(variables):
    """
    Display key variables in the console for debugging.
    Args:
        variables (dict): {name: value}
    Fires:
        Every frame (if debug enabled).
    """
    names = list(variables.keys())
    for i in range(len(names)):
        name = names[i]
        value = variables[name]
        print(f"{name}: {value}", end="\r\n")
    print("\033[F" * len(names), end="")