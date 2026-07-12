#!/usr/bin/python3
# File name   : Ultrasonic.py
# Description : Detection distance and tracking with ultrasonic
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/02/23
import RPi.GPIO as GPIO
import time

Tr = 11
Ec = 8

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(Ec, GPIO.IN)


def checkdist():       #Reading distance; -1.0 if the sensor does not respond
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(Ec, GPIO.IN)
    GPIO.output(Tr, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(Tr, GPIO.LOW)
    # a disconnected/faulty sensor never raises Ec — don't wait forever
    # (max range ~4 m = ~24 ms echo; 100 ms is generous)
    deadline = time.time() + 0.1
    while not GPIO.input(Ec):
        if time.time() > deadline:
            return -1.0
    t1 = time.time()
    deadline = t1 + 0.1
    while GPIO.input(Ec):
        if time.time() > deadline:
            return -1.0
    t2 = time.time()
    return round((t2-t1)*340/2,2)
    #return (t2-t1)*340/2

# def checkdist():       #Reading distance
#     GPIO.output(Tr, GPIO.HIGH)
#     time.sleep(0.000015)
#     GPIO.output(Tr, GPIO.LOW)
#     while not GPIO.input(Ec):
#         pass
#     t1 = time.time()
#     while GPIO.input(Ec):
#         t3 = time.time()
#         if ((t3-t1)*340/2)>=2:
#             break
#         pass
#     t2 = time.time()
#     return round((t2-t1)*340/2,2)

if __name__ == '__main__':
    while 1:
        print(checkdist())
        time.sleep(1)
