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

# pid_turn: face angle deg; pos is counter-clockwise
def pid_turn(deg: int):
    kP = 3
    kI = 0
    kD = 0.1
    integral = 0
    lastError = 0
    while deg - gyro.angle() != 0:
        error = deg - gyro.angle()
        integral += error
        derivative = error - lastError
        turn = kP * error + kI * integral + kD * derivative
        motor_b.run(turn)
        motor_c.run(-turn)
    motor_b.brake()
    motor_c.brake()

def pid_straight(dist: float, speed: int):
    kP = 3
    kI = 0.001
    kD = 0
    integral = 0
    lastError = 0
    heading = gyro.angle()
    in_per_rot = 10.21
    deg = (dist / in_per_rot) * 360
    motor_b.reset_angle(0)
    motor_c.reset_angle(0)
    while (motor_b.angle() + motor_c.angle()) / 2 > -deg:
        error = gyro.angle() - heading
        integral += error
        derivative = error - lastError
        turn = kP * error + kI * integral + kD * derivative
        motor_b.run(-speed - turn)
        motor_c.run(-speed + turn)
        lastError = error
    motor_b.brake()
    motor_c.brake()

def main():

    ev3.speaker.beep()

    gyro.reset_angle(0)
    motor_b.brake()
    motor_c.brake()
    pid_straight(80, 200)
    # wait(100)
    # pid_turn(-90)
    

if __name__ == "__main__":
    main()
