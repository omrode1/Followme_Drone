import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin to which the servo control wire is connected
servo_pin = 26

# Set the GPIO pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM (Pulse Width Modulation) instance with a frequency of 50Hz
pwm = GPIO.PWM(servo_pin, 50)

# Start PWM with a duty cycle of 2.5% (0 degrees)
pwm.start(2.5)

try:
    while True:
        # Move the servo to 90 degrees
        #pwm.ChangeDutyCycle(7.5)
        pwm.ChangeDutyCycle(4.0)
        time.sleep(1)

        # Move the servo back to 0 degrees
        pwm.ChangeDutyCycle(2.5)
        time.sleep(1)

except KeyboardInterrupt:
    # Stop PWM and clean up GPIO on Ctrl+C
    pwm.stop()
    GPIO.cleanup()
