#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


# Constants (meters)
WHEEL_DIAMETER = 0.056
WHEEL_RADIUS = WHEEL_DIAMETER / 2
AXLE_LENGTH = 0.092
RADIAN_CONVERSION = 180 / math.pi
TO_WALL_SPEED = 10 * RADIAN_CONVERSION
BACKUP_SPEED = (.17 / .056) * RADIAN_CONVERSION
FOLLOW_WALL_SPEED = 2 * RADIAN_CONVERSION
RIGHT_TURN_AV = 1.29 * (math.pi / 4) * (AXLE_LENGTH / (2 * WHEEL_RADIUS)) * RADIAN_CONVERSION
TARGET_DISTANCE_FROM_WALL = 0.12
K_P = 500 # proportional gain
MAX_SPEED = 4 * math.pi * RADIAN_CONVERSION


# EV3 objects
ev3 = EV3Brick()
motor_right = Motor(Port.C)
motor_left = Motor(Port.B)
ultrasonic_sensor = UltrasonicSensor(Port.S2)
bump_sensor = TouchSensor(Port.S1)


# Util Functions
def wait_for_center_button():
    while True:
        pressed_buttons = ev3.buttons.pressed()
        if Button.CENTER in pressed_buttons:
            break


def follow_wall(follow_distance):
    # Resets total degrees the motors have turned so far
    motor_left.reset_angle(0)
    motor_right.reset_angle(0)

    # Last degrees each motor rotated so far
    prev_right_angle = 0
    prev_left_angle = 0

    # Previous recorded distance from wall
    prev_distance_from_wall = ultrasonic_sensor.distance() / 1000

    total_distance_along_wall = 0

    while True:
        right_angle = motor_right.angle()
        left_angle = motor_left.angle()
        d_right_angle = math.radians(right_angle - prev_right_angle)
        d_left_angle = math.radians(left_angle - prev_left_angle)
        prev_right_angle = right_angle
        prev_left_angle = left_angle

        # Calculates distance traversed
        d_right_distance = WHEEL_RADIUS * d_right_angle
        d_left_distance = WHEEL_RADIUS * d_left_angle
        d_center_distance = (d_right_distance + d_left_distance) / 2

        # Determines wall angle
        curr_distance_from_wall = ultrasonic_sensor.distance() / 1000 # Convert mm to meters
        ev3.screen.print(curr_distance_from_wall)
        d_distance_from_wall = curr_distance_from_wall - prev_distance_from_wall
        prev_distance_from_wall = curr_distance_from_wall

        wall_angle = 0
        if abs(d_center_distance) > 1e-4:
            wall_angle = math.atan(d_distance_from_wall / d_center_distance)

        # Updates total distance along wall
        distance_along_wall = d_center_distance * math.cos(wall_angle)
        total_distance_along_wall += abs(distance_along_wall)

        if total_distance_along_wall >= follow_distance:
            motor_left.stop()
            motor_right.stop()
            ev3.speaker.beep()
            break

        error = TARGET_DISTANCE_FROM_WALL - curr_distance_from_wall

        correction = K_P * error

        left_speed = FOLLOW_WALL_SPEED + correction
        right_speed = FOLLOW_WALL_SPEED - correction

        max_speed = 720  # about 2 rotations/s
        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)

        motor_left.run(left_speed)
        motor_right.run(right_speed)

        wait(300)
        ev3.screen.clear()


        if Button.CENTER in ev3.buttons.pressed():
            ev3.speaker.beep()
            motor_left.stop()
            motor_right.stop()
            break


# Starts robot on button press
wait_for_center_button()
ev3.speaker.beep()

# Runs robot till wall until bumped
motor_right.run_time(TO_WALL_SPEED, 30_000, wait=False)
motor_left.run_time(TO_WALL_SPEED, 30_000, wait=False)

while not bump_sensor.pressed():
    wait(5)

motor_right.stop()
motor_left.stop()

# Runs robot back from wall after bumping
motor_right.run_time(-BACKUP_SPEED, 2_000, wait=False)
motor_left.run_time(-BACKUP_SPEED, 2_000)


# Turns robot right 90 degrees
motor_right.run_time(-RIGHT_TURN_AV, 2_000, wait=False)
motor_left.run_time(RIGHT_TURN_AV, 2_000)
wait(500)


# Follows wall for 2.2 meters
follow_wall(2.2)
