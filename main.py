#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

SQUARE_SIZE = 21

# action types
STRAIGHT = 1
TURN = 2

ev3 = EV3Brick()
gyro = GyroSensor(Port.S3)
motor_b = Motor(Port.B)
motor_c = Motor(Port.C)

# pid_turn: change orientation by deg; pos is counter-clockwise
def pid_turn(deg: int):
    kP = 5
    kI = 0.000
    kD = 0.1
    integral = 0
    lastError = 0

    # gyro is off by 1 degree every 90
    if deg < 0:
        deg += 1
    else:
        deg -= 1

    gyro.reset_angle(0)
    while gyro.angle() - deg != 0:
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

class Action():
    def __init__(self, action_type: int, value):
        self.action_type = action_type
        self.value = value

    def evaluate(self):
        if self.action_type == STRAIGHT:
            pid_straight(self.value, 600)
        elif self.action_type == TURN:
            pid_turn(int(self.value))

def evaluate_actions(actions: list):
    for action in actions:
        action.evaluate()
        wait(100)

def main():
    ev3.speaker.beep()

    gyro.reset_angle(0)
    motor_b.brake()
    motor_c.brake()
    actions = [
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, 90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE*2),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, 180),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, 90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, 180),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE*2),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE),
            Action(TURN, -90),
            Action(STRAIGHT, SQUARE_SIZE),
            ]
    evaluate_actions(actions)

if __name__ == "__main__":
    main()
