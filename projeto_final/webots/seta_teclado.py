import math
import time
from controller import Keyboard
from controller import Robot
from controller import Node

DISTANCE_BETWEEN_WHEELS = 0.381
WHEEL_RADIUS = 0.097
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
lmotor = robot.getDevice('left wheel')
rmotor = robot.getDevice('right wheel')
lmotor.setPosition(float('inf'))
rmotor.setPosition(float('inf'))
t = 0.0

#Rotacoes Completas
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
while robot.step(timestep) != -1:
    tn = robot.getTime()
    time = tn - t
    key = keyboard.getKey()
    
    if key == Keyboard.UP:  #sentido horário
        lmotor.setVelocity(10.0)
        rmotor.setVelocity(-10.0)
    elif key == Keyboard.DOWN: #sentido anti-horário
        lmotor.setVelocity(-10.0)
        rmotor.setVelocity(10.0)
    else:  #parar movimento
        lmotor.setVelocity(0.0)
        rmotor.setVelocity(0.0)
    pass
