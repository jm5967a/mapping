import time
from Motor import *
import RPi.GPIO as GPIO
from servo import *
import numpy as nm
import math
import sys
import threading

import PCA9685

nm.set_printoptions(threshold=sys.maxsize)

class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        self.trigger_pin = 27
        self.echo_pin = 22
        self.data_map = []
        self.map_size = (60, 60)
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
        i_left, i_center_left, i_center_right, i_right = nm.logical_or(self.data_map[0][1:-1], self.data_map[1][1:-1])

        if (i_left == 1 and i_center_left == 1 and i_center_right == 1 or i_right == 1) \
                or i_center_left == 1 and i_center_right == 1:
            self.PWM.setMotorModel(-1450, -1450, -1450, -1450)

            if (i_left == 1 or i_center_left == 1) and not (i_center_right == 1 or i_right == 1):
                self.PWM.setMotorModel(1450, 1450, -1450, -1450)
            else:
                self.PWM.setMotorModel(-1450, -1450, 1450, 1450)

        elif i_left == 1 and i_center_left == 1:
            PWM.setMotorModel(1500, 1500, -1500, -1500)
        elif i_right == 1 and i_center_right == 1:
            PWM.setMotorModel(-1500, -1500, 1500, 1500)
        elif i_left or i_center_left:
            PWM.setMotorModel(1500, 1500, -1000, -1000)
        elif i_right or i_center_right:
            PWM.setMotorModel(-1500, -1500, 1500, 1500)
        else:
            self.PWM.setMotorModel(600, 600, 600, 600)

    def run_motor2(self):
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
        tempMap = nm.zeros([60, 60])
        self.left = self.middle = self.right = 100

        for i in range(20, 160, 10):  # 34, 68, 102
            time.sleep(.05)
            self.pwm_S.setServoPwm('0', i)
            distance = self.get_distance()

            x = math.floor(math.cos(math.radians(i - 90)) * distance)
            y = math.floor(math.sin(math.radians(i - 90)) * distance)
            isAssumedClear = abs(y) >= 60 or abs(x) >= 60

            temp.append([x, y, isAssumedClear])

            if i == 30:
                self.left = distance
            elif i == 70 or i == 100:
                self.middle = min(self.middle, distance)
            elif i == 130:
                self.right = distance

        previousCordinates = None
        for cords in temp:
            x, y, isAssumedClear = cords

            if previousCordinates is not None and not isAssumedClear:
                prevX, prevY = previousCordinates
                for x_values in range(prevX, x):
                    for y_values in range(prevY, y):
                        print(x, prevX)

                        tempMap[y_values][x_values] = 1

            previousCordinates = None if isAssumedClear is True else [x, y]

        return tempMap

    def run(self):
        self.PWM = Motor()
        self.pwm_S = Servo()
        self.pwm_S.setServoPwm('1', 90)

        while True:
            self.data_map = self.map_world([0, 171, 34])
            self.run_motor2()

            nm.savetxt('file.txt', self.data_map)


ultrasonic = Ultrasonic()
# Main program logic follows:
if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        ultrasonic.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        PWM.setMotorModel(0, 0, 0, 0)
        ultrasonic.pwm_S.setServoPwm('0', 90)
