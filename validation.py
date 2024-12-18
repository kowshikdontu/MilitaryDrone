import threading
import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin, servo_pin):
        self.l = []
        self.uflag = True
        self.TRIG = trig_pin
        self.ECHO = echo_pin
        self.SERVO_PIN = servo_pin
        self.servo = True

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)

    def stop(self):
        self.uflag = False

    def rotate_servo(self, angle, p):
        duty_cycle = 2.5 + (angle / 18)
        GPIO.output(self.SERVO_PIN, True)
        p.ChangeDutyCycle(duty_cycle)
        time.sleep(0.2)
        GPIO.output(self.SERVO_PIN, False)

    def validate(self, dir, mov):
        if self.servo:
            pwm = GPIO.PWM(self.SERVO_PIN, 50)  # Servo typically uses 50Hz
            pwm.start(7.5)  # Neutral position

            d = {"forward": 0, "backward": 180, "right": 90, "left": 270}
            self.rotate_servo(d[dir], pwm)
            dist = int(self.get_distance())
            self.rotate_servo(0, pwm)  # Reset to initial position

            pwm.stop()

            if dist - mov > 10:
                return True
            else:
                return False
        else:
            dist = int(self.get_distance())
            if dist - mov > 10:
                return True
            else:
                return False

    def get_distance(self):
        try:
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            while GPIO.input(self.ECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(self.ECHO) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Convert to cm
            return round(distance, 2)

        except KeyboardInterrupt:
            GPIO.cleanup()

    def cleanup(self):
        GPIO.cleanup()


if __name__ == "__main__":
    try:
        trig = 23
        echo = 24
        servo = 25

        sensor = UltrasonicSensor(trig, echo, servo)

        while True:
            distance = sensor.get_distance()
            print(f"Distance: {distance} cm")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        sensor.cleanup()
