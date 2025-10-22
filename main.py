#!/usr/bin/env pybricks-micropython

# Leonardo Atalla (730605711)
# Abid Hussain (730664874)


from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port
from pybricks.parameters import Button
from pybricks.tools import wait
import math

WHEEL_DIAMETER = 56  # Wheel diameter in millimeters.
FRICTION = 1.05
TO_WALL_SPEED = FRICTION*(4*180)/math.pi
FOLLOW_WALL_SPEED = FRICTION*(3*180)/math.pi


def wait_for_center_button():
    while True:
        pressed_buttons = ev3.buttons.pressed()
        if Button.CENTER in pressed_buttons:
            break

ev3 = EV3Brick()

motor_right = Motor(Port.C)
motor_left = Motor(Port.B)

ultrasonic_sensor = UltrasonicSensor(Port.S2)
bump_sensor = TouchSensor(Port.S1)


ev3.speaker.beep()

wait_for_center_button()

# Play a sound.
ev3.speaker.beep()

motor_right.run_time(TO_WALL_SPEED, 30_000, wait=False)
motor_left.run_time(TO_WALL_SPEED, 30_000, wait=False)

while not bump_sensor.pressed():
    wait(10)

motor_right.stop()
motor_left.stop()

ev3.speaker.beep()

# Go back 15cm
# Run motors to go back in 3 seconds
backwards_speed = (180/math.pi)*( (0.15 / 3) / (WHEEL_DIAMETER / 2))
motor_right.run_time(-backwards_speed, 3_000, wait=False)
motor_left.run_time(-backwards_speed, 3_000, wait=False)

wait(3_000)
ev3.speaker.beep()

# turn 90 degrees to the right
turn_speed = FRICTION*90/1_500
motor_right.run_time(turn_speed, 1_500, wait=False)
motor_left.run_time(-turn_speed, 1_500, wait=False)
wait(1_500)
ev3.speaker.beep()

# follow the wall until have travelled 2.2 meters in positive x direction
distance_travelled = 0
while distance_travelled < 2_200:
    distance_to_wall = ultrasonic_sensor.distance()
    error = distance_to_wall - 250  # target distance is 250 mm
    Kp = 1.2  # Proportional gain
    correction = Kp * error

    left_speed = FOLLOW_WALL_SPEED + correction
    right_speed = FOLLOW_WALL_SPEED - correction

    motor_left.run(left_speed)
    motor_right.run(right_speed)

    # ensure to update distance_travelled only in the positive x direction, none in the y direction
    if left_speed > right_speed:
        distance_travelled += (left_speed * 10) / 1000  # Convert mm/s to mm per 10 ms
    else:
        distance_travelled += (right_speed * 10) / 1000  # Convert mm/s to mm per 10 ms

    wait(10)
motor_right.stop()
motor_left.stop()

ev3.speaker.beep()