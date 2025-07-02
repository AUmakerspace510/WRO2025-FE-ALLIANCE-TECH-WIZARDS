import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor pins
IN1 = 17
IN2 = 27
ENA = 19

# Servo pin
SERVO_PIN = 18

# Ultrasonic pins
TRIG_F = 5
ECHO_F = 6

TRIG_L = 13
ECHO_L = 16

TRIG_R = 20
ECHO_R = 21

# Setup
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(SERVO_PIN, GPIO.OUT)

GPIO.setup(TRIG_F, GPIO.OUT)
GPIO.setup(ECHO_F, GPIO.IN)
GPIO.setup(TRIG_L, GPIO.OUT)
GPIO.setup(ECHO_L, GPIO.IN)
GPIO.setup(TRIG_R, GPIO.OUT)
GPIO.setup(ECHO_R, GPIO.IN)

motor_pwm = GPIO.PWM(ENA, 1000)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)

motor_pwm.start(0)
servo_pwm.start(0)

def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.4)
    servo_pwm.ChangeDutyCycle(0)

def move_forward(speed=100):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed)

def move_backward(speed=100):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(speed)

def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)

def get_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.01)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

try:
    set_servo_angle(110)  # Center

    while True:
        front_dist = get_distance(TRIG_F, ECHO_F)
        left_dist = get_distance(TRIG_L, ECHO_L)
        right_dist = get_distance(TRIG_R, ECHO_R)

        print(f"Front: {front_dist} cm, Left: {left_dist} cm, Right: {right_dist} cm")

        if front_dist > 50:
            set_servo_angle(110)
            move_forward(80)
        else:
            stop_motor()
            time.sleep(0.2)

            if left_dist > right_dist and left_dist > 40:
                print("Obstacle in front! Turning left.")
                set_servo_angle(60)
                move_forward(80)
                time.sleep(0.7)
            elif right_dist > left_dist and right_dist > 40:
                print("Obstacle in front! Turning right.")
                set_servo_angle(160)
                move_forward(80)
                time.sleep(0.7)
            else:
                # If no side is clear enough, move backward slightly
                print("No clear path, moving backward")
                set_servo_angle(110)
                move_backward(80)
                time.sleep(0.5)

            stop_motor()
            set_servo_angle(110)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by User")

finally:
    stop_motor()
    motor_pwm.stop()
    servo_pwm.stop()
    GPIO.cleanup()
