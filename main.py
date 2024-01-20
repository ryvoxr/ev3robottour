#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()
gyro = GyroSensor(Port.S3)
motor_b = Motor(Port.B)
motor_c = Motor(Port.C)

def pid_straight(deg, speed):
    kP = 4
    kI = 0.001
    kD = 0.001
    integral = 0
    lastError = 0
    gyro.reset_angle(0)
    while (motor_b.angle() + motor_c.angle()) / 2 > -deg:
        error = gyro.angle()
        integral += error
        derivative = error - lastError
        turn = kP * error + kI * integral + kD * derivative
        motor_b.run(-speed + turn)
        motor_c.run(-speed - turn)
        lastError = error
    motor_b.brake()
    motor_c.brake()

def main():

    ev3.speaker.beep()

    gyro.reset_angle(0)
    motor_b.brake()
    motor_c.brake()
    pid_straight(3600, 200)
    

if __name__ == "__main__":
    main()
