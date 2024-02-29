"""my_controller2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
#import keyboard
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
robot.step(timestep)

rmotor.setPosition(10) #Posição objetivo
lmotor.setPosition(10)
    
rmotor.setVelocity(5) #Velocidade
lmotor.setVelocity(5)

rmotor.setAcceleration(2) #Aceleração 
lmotor.setAcceleration(2)
# Main loop:
#while robot.step(timestep) != -1:

    #lmotor.setControlPID(1,0.1,0.01)
    #rmotor.setControlPID(1,0.1,0.01)

while robot.step(timestep) != -1:

    lmotor.setControlPID(2,0.2,0.02)
    rmotor.setControlPID(2,0.2,0.02)

