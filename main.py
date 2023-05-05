#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Create your objects here.
ev3 = EV3Brick()
rotation_motor = Motor(Port.A)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
us_sensor = UltrasonicSensor(Port.S4)
gyro_sensor = GyroSensor(Port.S3)

# Write your program here.
ev3.speaker.beep()

# Initialisation des variables
stop_distance = 200
slow_distance = stop_distance + 30
turn_speed = 300
fast_forward_speed = 400
slow_forward_speed = 200

# Fonction pour obtenir la distance de l'obstacle en face
def get_distance():
    return us_sensor.distance()

# Fonction pour tourner à gauche
def turn_left():
    left_motor.run_angle(-turn_speed, 360, then=Stop.COAST, wait=False)
    right_motor.run_angle(turn_speed, 360, then=Stop.COAST, wait=True)

# Fonction pour tourner à droite
def turn_right():
    left_motor.run_angle(turn_speed, 360, then=Stop.COAST, wait=False)
    right_motor.run_angle(-turn_speed, 360, then=Stop.COAST, wait=True)

# Fonction pour faire tourner le moteur de rotation
def rotate_motor(rotation_angle):
    rotation_motor.run_angle(turn_speed, rotation_angle, then=Stop.COAST, wait=True)
    wait(500)  # Attendre une demi-seconde pour que le moteur s'arrête complètement
    rotation_motor.run_angle(-turn_speed, rotation_angle, then=Stop.COAST, wait=True) # Tourner le moteur dans le sens inverse pour revenir à son état initial

# Fonction pour obtenir la position actuelle du robot
def get_position():
    return gyro_sensor.angle()

# Fonction pour faire fonctionner le capteur UltrasonicSensor comme dans le code 2
def get_front_distance():
    motor_left.run(fast_forward_speed)
    motor_right.run(fast_forward_speed)
    wait(500) # Attendre que le robot soit bien en mouvement
    distance = us_sensor.distance()
    motor_left.stop()
    motor_right.stop()
    wait(500) # Attendre que le robot s'arrête complètement
    return distance

# Continuously update position on screen
while True:
    # Get current position from GyroSensor
    position = get_position()

    # Display position on screen
    ev3.screen.clear()
    ev3.screen.draw_text(50, 50, "Position: {}".format(position))

# Check distance to obstacle and take appropriate action
distance = get_distance()

if distance <= stop_distance or door1_sensor.pressed() or door2_sensor.pressed():
    left_motor.stop()
    right_motor.stop()
    ev3.speaker.beep()
    
    # Check which door sensor is pressed
    if door1_sensor.pressed():
        print("Door 1 is blocked!")
    elif door2_sensor.pressed():
        print("Door 2 is blocked!")
        
    # Reversing to create space in front of the robot
    left_motor.run_to_rel_pos(position_sp=-200, speed_sp=300, stop_action='brake')
    right_motor.run_to_rel_pos(position_sp=-200, speed_sp=300, stop_action='brake')
    
    # Turn around
    left_motor.run_to_rel_pos(position_sp=-550, speed_sp=300, stop_action='brake')
    right_motor.run_to_rel_pos(position_sp=550, speed_sp=300, stop_action='brake')
    
    # Continue moving forward
    left_motor.run_to_rel_pos(position_sp=1000, speed_sp=300, stop_action='brake')
    right_motor.run_to_rel_pos(position_sp=1000, speed_sp=300, stop_action='brake')
    # No obstacles detected, continue moving forward
else:
    left_motor.run_forever(speed_sp=300)
    right_motor.run_forever(speed_sp=300)

    # Check if we have arrived at our destination
    if arrived_at_destination():
        left_motor.stop()
        right_motor.stop()
        ev3.speaker.say("Arrived at destination")

    # Check if we need to turn left or right
    elif need_to_turn():
        # Stop the motors and turn
        left_motor.stop()
        right_motor.stop()
        ev3.speaker.say("Turning")
        turn()

    # Check if we need to stop at a traffic light
    elif at_traffic_light():
        # Stop the motors and wait for the light to turn green
        left_motor.stop()
        right_motor.stop()
        ev3.speaker.say("Waiting for green light")
        wait_for_green_light()

    # Check if we need to stop at a pedestrian crossing
    elif at_pedestrian_crossing():
        # Stop the motors and wait for the pedestrians to cross
        left_motor.stop()
        right_motor.stop()
        ev3.speaker.say("Waiting for pedestrians to cross")
        wait_for_pedestrians()
    #After reaching the end of the hallway, turn around and go back
left_motor.run_to_rel_pos(position_sp=-500, speed_sp=300)
right_motor.run_to_rel_pos(position_sp=500, speed_sp=300)
while left_motor.is_running() or right_motor.is_running():
pass
    
After reaching the end of the hallway, turn around and go back
left_motor.run_to_rel_pos(position_sp=-500, speed_sp=300)
right_motor.run_to_rel_pos(position_sp=500, speed_sp=300)
while left_motor.is_running() or right_motor.is_running():
pass

#Move forward until obstacle is detected
while True:
distance = get_distance()
if distance <= stop_distance or door1_sensor.pressed() or door2_sensor.pressed():
left_motor.stop()
right_motor.stop()
ev3.speaker.beep()
break
left_motor.run_forever(speed_sp=300)
right_motor.run_forever(speed_sp=300)
#Turn around and go back again
left_motor.run_to_rel_pos(position_sp=-500, speed_sp=300)
right_motor.run_to_rel_pos(position_sp=500, speed_sp=300)
while left_motor.is_running() or right_motor.is_running():
pass
#Move forward until obstacle is detected
while True:
distance = get_distance()
if distance <= stop_distance or door1_sensor.pressed() or door2_sensor.pressed():
left_motor.stop()
right_motor.stop()
ev3.speaker.beep()
break
left_motor.run_forever(speed_sp=300)
right_motor.run_forever(speed_sp=300)

#Program finished
ev3.speaker.say("I have reached the end of the hallway and returned twice.")
