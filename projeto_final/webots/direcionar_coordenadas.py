"""direcionar_coordenadas controller."""
import math
import time
from controller import Keyboard
from controller import Robot, Supervisor
from controller import Node

#Em malha aberta
robot = Robot()
DISTANCE_BETWEEN_WHEELS = 0.381
WHEEL_RADIUS = 0.097
timestep = int(robot.getBasicTimeStep())
lmotor = robot.getDevice('left wheel')
rmotor = robot.getDevice('right wheel')
lmotor.setPosition(float('inf'))
rmotor.setPosition(float('inf'))
lmotor.setVelocity(1.0)
rmotor.setVelocity(1.0)
t = robot.getTime()

while robot.step(timestep) !=-1:
    time = robot.getTime() - t
    if time <= 0.95:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(-1.0)
        print('lmotor1:', lmotor)
        print('rmotor1:', rmotor)
        print('tempo1:', time)
    elif time > 0.95 and time < 26.2:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(1.0)
        print('lmotor2:', lmotor)
        print('rmotor2:', rmotor)
        print('tempo2:', time)
    else:
        rmotor.setVelocity(0.0)
        lmotor.setVelocity(0.0)
pass

