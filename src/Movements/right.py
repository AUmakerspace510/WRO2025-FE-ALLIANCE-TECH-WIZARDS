import RPi.GPIO as GPIO
import time

# GPIO pin setup
GPIO.setmode(GPIO.BCM)

# L298N motor pins
IN1 = 17
IN2 = 27
ENA = 19

# Servo pin
SERVO_PIN = 18

# Setup motor pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Setup servo pin
GPIO.setup(SERVO_PIN, GPIO.OUT)

# PWM setup
motor_pwm = GPIO.PWM(ENA, 1000)      # Motor PWM at 1kHz
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # Servo PWM at 50Hz

motor_pwm.start(0)
servo_pwm.start(0)

# Function to set servo angle
def set_servo_angle(angle):
    duty = 2 + (angle / 18)  # Convert angle to duty cycle
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)  # Stop sending signal (avoid jitter)

# Functions to move motor
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

try:

    # Turn right and move forward
    set_servo_angle(152)
    move_forward(100)
    time.sleep(3)    
    stop_motor()

except KeyboardInterrupt:
    pass

finally:
    print("Cleaning up")
    stop_motor()
    motor_pwm.stop()
    servo_pwm.stop()
    GPIO.cleanup()
