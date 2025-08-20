import time
import board
import busio
import adafruit_tcs34725
import RPi.GPIO as GPIO
import threading
from collections import deque

# === GPIO SETUP ===
GPIO.setmode(GPIO.BCM)

# Motor pins
IN1, IN2, ENA = 17, 27, 19
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Servo pin
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)


# Ultrasonic pins
TRIG_LEFT, ECHO_LEFT = 13, 16
TRIG_FRONT, ECHO_FRONT = 5, 6
TRIG_RIGHT, ECHO_RIGHT = 20, 21
for trig, echo in [(TRIG_LEFT, ECHO_LEFT), (TRIG_FRONT, ECHO_FRONT), (TRIG_RIGHT, ECHO_RIGHT)]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)

# LED Control for Color Sensor
LED_PIN = 12
GPIO.setup(LED_PIN, GPIO.OUT)
led_pwm = GPIO.PWM(LED_PIN, 10000)  # 10kHz frequency for stable LED brightness
led_pwm.start(0)  # Start with LED OFF

# Motor PWM
motor_pwm = GPIO.PWM(ENA, 1000)
motor_pwm.start(0)

# Servo PWM
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)

# Color sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 24  # Better accuracy than 10ms
sensor.gain = 4  # Balanced gain

# Set LED brightness for color detection
led_pwm.ChangeDutyCycle(15)  # Adjust LED intensity if needed

# === Servo Control ===
def set_angle(angle):
    duty = 2 + (angle / 18)
    servo_pwm.ChangeDutyCycle(duty)

ANGLE_CENTER = 110

# === Motor Control ===
def motor_forward(speed=40):
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    motor_pwm.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    motor_pwm.ChangeDutyCycle(0)

# === Ultrasonic with Filtering ===
distances = {"left": 100, "front": 100, "right": 100}
lock = threading.Lock()

# Buffers for moving average
dist_buffers = {
    "left": deque(maxlen=5),
    "front": deque(maxlen=5),
    "right": deque(maxlen=5),
}

def read_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.0002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start, end = time.time(), time.time()
    timeout = time.time() + 0.03

    while GPIO.input(echo) == 0 and time.time() < timeout:
        start = time.time()
    while GPIO.input(echo) == 1 and time.time() < timeout:
        end = time.time()

    duration = end - start
    distance = duration * 17150

    if distance <= 0 or distance > 200:
        return None
    return round(distance, 1)

def ultrasonic_manager():
    while True:
        for name, trig, echo in [
            ("left", TRIG_LEFT, ECHO_LEFT),
            ("front", TRIG_FRONT, ECHO_FRONT),
            ("right", TRIG_RIGHT, ECHO_RIGHT),
        ]:
            d = read_distance(trig, echo)
            if d is not None:
                dist_buffers[name].append(d)
                with lock:
                    distances[name] = sum(dist_buffers[name]) / len(dist_buffers[name])
            time.sleep(0.03)

# === Updated Color Detection ===
def detect_color(r, g, b):
    # Orange range
    if 40 <= r <= 70 and 25 <= g <= 50 and 15 <= b <= 35:
        return "Orange"
    # Blue range
    elif 9 <= r <= 25 and 15 <= g <= 30 and 15 <= b <= 30:
        return "Blue"
    else:
        return "Unknown"

# === PID Wall Following ===
def wall_follow(mode, turn_counter):
    print(f"Wall following mode: {mode}")
    desired = 20
    Kp, Kd = 2.0, 1.0
    prev_error = 0
    last_turn_time = 0       # timestamp of last turn
    cooldown = 2.0           # seconds to ignore re-detection

    while turn_counter[0] <= 4:  # Stop after 4 valid turns
        print(turn_counter[0])
        with lock:
            d_left, d_front, d_right = distances["left"], distances["front"], distances["right"]

        if d_front < 20:
            print("Obstacle ahead! Stopping.")
            motor_stop()
            break

        if mode == "left":
            error = desired - d_left
        else:
            error = d_right - desired

        derivative = error - prev_error
        correction = Kp * error + Kd * derivative
        prev_error = error

        angle = ANGLE_CENTER + correction
        angle = max(55, min(155, angle))
        set_angle(angle)

        motor_forward(40)

        # --- Color Detection with Cooldown ---
        now = time.time()
        if now - last_turn_time > cooldown:
            r, g, b, _ = sensor.color_raw
            color_name1 = detect_color(r, g, b)  # only check if cooldown passed
            if mode == "left" and color_name1 == "Blue":
                print("Valid LEFT turn detected!")
                do_turn("left")
                turn_counter[0] += 1
                last_turn_time = now
            elif mode == "right" and color_name1 == "Orange":
                print("Valid RIGHT turn detected!")
                do_turn("right")
                turn_counter[0] += 1
                last_turn_time = now
            color_name1 = ""

        time.sleep(0.05)

    print("4 turns completed. Stopping.")
    motor_stop()

# === Turn Handling ===
def do_turn(direction):
    if direction == "left":
        set_angle(55)
    else:
        set_angle(155)

    motor_forward(50)
    time.sleep(2)   # turning duration
    set_angle(ANGLE_CENTER)

# === MAIN ===
try:
    print("Phase 1: Moving forward, waiting for first color...")
    motor_forward(40)
    set_angle(ANGLE_CENTER)

    follow_mode = None
    while True:
        r, g, b, _ = sensor.color_raw
        color_name = detect_color(r, g, b)
        print(f"R:{r}, G:{g}, B:{b} -> {color_name}")

        if color_name == "Blue":
            print("Blue detected → steer LEFT")
            do_turn("left")
            follow_mode = "left"
            break
        elif color_name == "Orange":
            print("Orange detected → steer RIGHT")
            do_turn("right")
            follow_mode = "right"
            break
        elif color_name == "Unknown":
            print("Unknown color detected. Continuing to move forward.")

        time.sleep(0.1)

    print("Phase 2: Starting wall following...")
    threading.Thread(target=ultrasonic_manager, daemon=True).start()

    # Use list as mutable counter
    wall_follow(follow_mode, [0])

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    motor_stop()
    servo_pwm.stop()
    motor_pwm.stop()
    led_pwm.stop()
    GPIO.cleanup()
    print("Clean exit.")