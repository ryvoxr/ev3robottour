#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# target time in seconds
TARGET_TIME = 54


SS = 20.25  # square size
DRV = 1     # drive action
TRN = 2     # turn action
R = -90     # right turn
L = 90      # left turn

IN_PER_ROT = 10.21

ev3 = EV3Brick()
gyro = GyroSensor(Port.S3)
motor_b = Motor(Port.B)
motor_c = Motor(Port.C)
timer = StopWatch()

# pid_turn: change orientation by deg; pos is counter-clockwise
def pid_turn(deg: int):
    kP = 4
    kI = 0.000
    kD = 0.2
    integral = 0
    lastError = 0

    gyro.reset_angle(0)
    heading = gyro.angle()
    target = heading + deg
    while abs(target - gyro.angle()) != 0:
        error = target - gyro.angle()
        integral += error
        derivative = error - lastError
        turn = kP * error + kI * integral + kD * derivative
        motor_b.run(-turn)
        motor_c.run(turn)
    motor_b.brake()
    motor_c.brake()

def pid_straight(dist: float, speed: int):
    kP = 2.5
    kI = 0.001
    kD = 0
    integral = 0
    lastError = 0

    gyro.reset_angle(0)
    deg = (dist / IN_PER_ROT) * 360
    motor_b.reset_angle(0)
    motor_c.reset_angle(0)
    while abs(motor_b.angle() + motor_c.angle()) / 2 < abs(deg):
        error = gyro.angle()
        integral += error
        derivative = error - lastError
        turn = kP * error + kI * integral + kD * derivative
        motor_b.run(speed + turn)
        motor_c.run(speed - turn)
        lastError = error
    motor_b.brake()
    motor_c.brake()

class Action():
    def __init__(self, action_type: int, value):
        self.action_type = action_type
        self.value = value

    def evaluate(self):
        if self.action_type == DRV:
            pid_straight(self.value, 600)
        elif self.action_type == TRN:
            pid_turn(int(self.value))

def evaluate_actions(actions: list):
    for action in actions:
        action.evaluate()
        wait(100)

def final_drive():
    time_remaining = TARGET_TIME - timer.time()/1000
    if time_remaining < 0:
        pid_straight(SS*.85, 600)
        return

    ang_vel = (SS*.85 * 360) / (IN_PER_ROT * time_remaining)
    pid_straight(SS*.85, int(ang_vel))
    while timer.time()/1000 < TARGET_TIME:
        motor_b.run(5)
        motor_c.run(5)

def first_drive():
    pid_straight(SS*0.25, -300)
    wait(100)

def main():
    ev3.speaker.beep()

    gyro.reset_angle(0)
    motor_b.brake()
    motor_c.brake()
    actions = [
            Action(TRN, R),
            Action(DRV, SS*2),
            Action(TRN, 180),
            Action(DRV, SS*2),
            Action(TRN, L),
            Action(DRV, SS*3),
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS*2),
            Action(TRN, 180),
            Action(DRV, SS*2),
            Action(TRN, L),
            Action(DRV, SS),
            Action(TRN, L),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, L),
            Action(DRV, SS),
            Action(TRN, L),
            Action(DRV, SS),
            Action(TRN, 180),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, L),
            Action(DRV, SS),
            Action(TRN, R),
            ]
    timer.reset()
    first_drive()
    evaluate_actions(actions)
    final_drive()

if __name__ == "__main__":
    main()
