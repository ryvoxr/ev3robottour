#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# target time in seconds
TARGET_TIME = 45


SS = 20.25  # square size
DRV = 1     # drive action
TRN = 2     # turn action
R = -90     # right turn
L = 90      # left turn

IN_PER_ROT = 10.21
TURNING_TIME = 3
FINAL_DRIVE_DIST = 0.85*SS

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
    deg = (dist / IN_PER_ROT) * 360

    # set direction
    if speed < 0:
        forward = False
        speed = -speed
    else:
        forward = True

    gyro.reset_angle(0)
    motor_b.reset_angle(0)
    motor_c.reset_angle(0)
    currentAngle = 0
    while currentAngle < deg:
        currentAngle = abs((motor_b.angle() + motor_c.angle()) / 2)

        # do PID stuff
        error = gyro.angle()
        integral += error
        derivative = error - lastError
        turn = kP * error + kI * integral + kD * derivative

        # drive motors
        if forward:
            motor_b.run(speed + turn)
            motor_c.run(speed - turn)
        else:
            motor_b.run(-speed + turn)
            motor_c.run(-speed - turn)
    motor_b.brake()
    motor_c.brake()

class Action():
    def __init__(self, action_type: int, value):
        self.action_type = action_type
        self.value = value

    def evaluate(self, speed: int):
        if self.action_type == DRV:
            pid_straight(self.value, speed)
        elif self.action_type == TRN:
            pid_turn(int(self.value))

class ActionQueue():
    def __init__(self, actions: list):
        self.actions = actions
        self.current_action = 0
        self.turning_time_remaining = self.get_turning_time()
        self.driving_distance_remaining = self.get_driving_distance()
        self.speed = 0
        self.update_speed()

    def run(self):
        while (self.next_action()):
            wait(100)

    def next_action(self):
        if self.current_action < len(self.actions):
            self.actions[self.current_action].evaluate(self.speed)
            self.update_remaining(self.actions[self.current_action])
            self.update_speed()
            self.current_action += 1
            return True
        else:
            return False

    def update_speed(self):
        self.speed = self.driving_distance_remaining / ((TARGET_TIME - timer.time()/1000 - 2) - self.turning_time_remaining)

    def update_remaining(self, action: Action):
        if action.action_type == DRV:
            self.driving_distance_remaining -= ((action.value / IN_PER_ROT) * 360)
        elif action.action_type == TRN:
            self.turning_time_remaining -= TURNING_TIME

    def get_turning_time(self):
        time = 0
        for action in self.actions:
            if action.action_type == TRN:
                time += TURNING_TIME
        return time

    def get_driving_distance(self):
        dist = 0
        for action in self.actions:
            if action.action_type == DRV:
                dist += action.value
        dist += FINAL_DRIVE_DIST
        deg = (dist / IN_PER_ROT) * 360
        return deg


def final_drive():
    time_remaining = TARGET_TIME - timer.time()/1000
    if time_remaining < 0:
        pid_straight(FINAL_DRIVE_DIST, 600)
        return

    ang_vel = (FINAL_DRIVE_DIST * 360) / (IN_PER_ROT * time_remaining)
    pid_straight(FINAL_DRIVE_DIST, int(ang_vel))
    while timer.time()/1000 < TARGET_TIME:
        motor_b.run(5)
        motor_c.run(5)

def first_drive():
    pid_straight(SS*0.35, -300)
    wait(100)

def main():
    ev3.speaker.beep()

    gyro.reset_angle(0)
    motor_b.brake()
    motor_c.brake()
    actions = [
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS*3),
            Action(TRN, R),
            Action(DRV, SS*2),
            Action(TRN, 180),
            Action(DRV, SS*2),
            Action(TRN, L),
            Action(DRV, SS),
            Action(TRN, R),
            Action(DRV, SS),
            Action(TRN, L),
            ]
    timer.reset()
    first_drive()
    action_queue = ActionQueue(actions)
    action_queue.run()
    final_drive()

if __name__ == "__main__":
    main()
