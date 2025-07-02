import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import threading

# ================= GPIO Setup ===================
GPIO.setmode(GPIO.BCM)

# Motor and Servo pins
IN1 = 17
IN2 = 27
ENA = 19
SERVO_PIN = 18

# Ultrasonic pins
TRIG = 5
ECHO = 6

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

motor_pwm = GPIO.PWM(ENA, 1000)  # Motor speed control
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz for servo

motor_pwm.start(0)
servo_pwm.start(0)

# ========== Threaded Video Class ==============
class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            print("[Error] Cannot open camera")
            GPIO.cleanup()
            exit()
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.cap.release()

# ================ Functions =================

def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    print(f"[Servo] Setting angle: {angle} -> Duty: {duty:.2f}")
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)
    servo_pwm.ChangeDutyCycle(0)

def move_forward(speed=80):
    print(f"[Motor] Moving forward with speed: {speed}")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed)

def stop_motor():
    print("[Motor] Stopping")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.01)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    timeout = time.time() + 0.04  # 40ms timeout

    while GPIO.input(ECHO) == 0:
        if time.time() > timeout:
            return -1
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        if time.time() > timeout:
            return -1
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound in cm/s

    if distance <= 0 or distance > 300:
        return -1  # Invalid range
    return round(distance, 2)

def average_distance(samples=3):
    distances = []
    for _ in range(samples):
        d = get_distance()
        if d != -1:
            distances.append(d)
        time.sleep(0.01)
    if distances:
        return round(sum(distances) / len(distances), 2)
    else:
        return -1

# ========== Start video capture thread ==========
cap = VideoStream(0)
time.sleep(1)  # Allow camera to warm up

# ========== Motor test before main loop ==========
print("=== Running motor test ===")
move_forward(80)
time.sleep(2)
stop_motor()
time.sleep(1)
print("=== Motor test done ===")

# ========== Main loop variables ==========
state = "normal"
maneuver_start = 0
step = 0

try:
    set_servo_angle(112)
    move_forward(80)

    while True:
        ret, frame = cap.read()

        if not ret:
            print("[Error] Camera frame not received")
            stop_motor()
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red detection
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        # Green detection
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        red_detected = cv2.countNonZero(mask_red) > 5000
        green_detected = cv2.countNonZero(mask_green) > 5000

        current_time = time.time()

        if state == "normal":
            set_servo_angle(112)
            move_forward(80)

            distance = average_distance()
            print(f"[Ultrasonic] Distance: {distance} cm")

            if 0 < distance < 30:
                if red_detected:
                    print("[State] Red detected! Start right-left sequence")
                    state = "red"
                    maneuver_start = current_time
                    step = 0
                elif green_detected:
                    print("[State] Green detected! Start left-right sequence")
                    state = "green"
                    maneuver_start = current_time
                    step = 0

        elif state == "red":
            if step == 0:
                print("[Red] Step 1: Turning right")
                set_servo_angle(142)
                move_forward(80)
                maneuver_start = current_time
                step = 1
            elif step == 1 and current_time - maneuver_start > 1:
                print("[Red] Step 2: Turning left")
                set_servo_angle(82)
                move_forward(80)
                maneuver_start = current_time
                step = 2
            elif step == 2 and current_time - maneuver_start > 1:
                print("[Red] Sequence complete. Returning to normal.")
                state = "normal"

        elif state == "green":
            if step == 0:
                print("[Green] Step 1: Turning left")
                set_servo_angle(72)
                move_forward(100)
                maneuver_start = current_time
                step = 1
            elif step == 1 and current_time - maneuver_start > 1:
                print("[Green] Step 2: Turning right")
                set_servo_angle(152)
                move_forward(80)
                maneuver_start = current_time
                step = 2
            elif step == 2 and current_time - maneuver_start > 1:
                print("[Green] Sequence complete. Returning to normal.")
                state = "normal"

        # Show reduced-size preview window for speed
        resized_frame = cv2.resize(frame, (320, 240))
        cv2.imshow("Live Feed", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[User] Quit command received.")
            break

except KeyboardInterrupt:
    print("[User] Keyboard interrupt received.")

finally:
    print("[Cleanup] Stopping and cleaning up GPIO and camera.")
    stop_motor()
    motor_pwm.stop()
    servo_pwm.stop()
    GPIO.cleanup()
    cap.stop()
    cv2.destroyAllWindows()