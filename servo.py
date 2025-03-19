from gpiozero import AngularServo 
from time import sleep

servo = AngularServo(4, min_pulse_width=0.0006, max_pulse_width=0.0023)

while (True):
    servo.angle = 90
    sleep (1)
    servo.angle = 0
    sleep (1)
    servo.angle = -89
    sleep (1)
