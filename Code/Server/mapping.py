import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
import numpy as nm
import math
import sys

class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        self.data_map = []
        self.map_size = (30, 30)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def send_trigger_pulse(self):
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00015)
        GPIO.output(self.trigger_pin, False)

    def wait_for_echo(self, value, timeout):
        count = timeout
        while GPIO.input(self.echo_pin) != value and count > 0:
            count = count - 1

    def get_distance(self):
        distance_cm = [0, 0, 0, 0, 0]
        for i in range(3):
            self.send_trigger_pulse()
            self.wait_for_echo(True, 10000)
            start = time.time()
            self.wait_for_echo(False, 10000)
            finish = time.time()
            pulse_len = finish - start
            distance_cm[i] = pulse_len / 0.000058
        distance_cm = sorted(distance_cm)
        return int(distance_cm[2])

    def run_motor(self):
        L = self.left
        M = self.middle
        R = self.right

        if (L < 30 and M < 30 and R < 30) or M < 30:
            self.PWM.setMotorModel(-1450, -1450, -1450, -1450)
            time.sleep(0.1)
            if L < R:
                self.PWM.setMotorModel(1450, 1450, -1450, -1450)
            else:
                self.PWM.setMotorModel(-1450, -1450, 1450, 1450)
        elif L < 30 and M < 30:
            PWM.setMotorModel(1500, 1500, -1500, -1500)
        elif R < 30 and M < 30:
            PWM.setMotorModel(-1500, -1500, 1500, 1500)
        elif L < 20:
            PWM.setMotorModel(2000, 2000, -500, -500)
            if L < 10:
                PWM.setMotorModel(1500, 1500, -1000, -1000)
        elif R < 20:
            PWM.setMotorModel(-500, -500, 2000, 2000)
            if R < 10:
                PWM.setMotorModel(-1500, -1500, 1500, 1500)
        else:
            self.PWM.setMotorModel(600, 600, 600, 600)

    def map_world(self):
        temp = []
        tempMap = nm.zeros([30, 80], dtype=int).astype(int)
        self.left = self.middle = self.right = 100

        for i in range(0, 170, 10):  # 20 30 40 50 60 70 80
            time.sleep(.2)
            self.pwm_S.setServoPwm('0', i)
            distance = self.get_distance()

            testing = math.radians(i)
            x = math.floor(math.cos(testing) * distance) + 40
            y = math.floor(math.sin(testing) * distance // 10)

            isAssumedClear = abs(y) >= 30 or abs(x) >= 80
            temp.append([x, y, isAssumedClear])

            if i == 30:
                self.left = distance
            elif i == 70 or i == 100:
                self.middle = min(self.middle, distance)
            elif i == 130:
                self.right = distance

        temp.sort(key=lambda itm: itm[0])
        pointer = 0

        while pointer < len(temp):
            if not temp[pointer][2]:
                x, y, _ = temp[pointer]
                tempMap[y][x] = 5

                if len(temp) != pointer + 1 and temp[pointer + 1] and not temp[pointer + 1][2] and temp[pointer + 1][1] < y + 2:
                    x_val, y_val, _ = temp[pointer + 1]
                    for x_cords in range(x + 1, x_val):
                        tempMap[y][x_cords] = 1

            pointer += 1

        nm.set_printoptions(threshold=sys.maxsize, precision=3, linewidth=350)
        print("")
        print(tempMap)
        return tempMap

    def run(self):
        self.PWM = Motor()
        self.pwm_S = Servo()
        self.pwm_S.setServoPwm('1', 90)

        while True:
            self.data_map = self.map_world()
            self.run_motor()

ultrasonic = Ultrasonic()
# Main program logic follows:
if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        ultrasonic.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        PWM.setMotorModel(0, 0, 0, 0)
        ultrasonic.pwm_S.setServoPwm('0', 90)
