"""planeamento_aberto controller."""
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


while robot.step(timestep) != -1:
    time = robot.getTime() - t
    if time <= 2.7:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(-1.0)
        print('lmotor1:', lmotor)
        print('rmotor1:', rmotor)
        print('tempo1:', time)
    elif time > 2.7 and time <= 21.75:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(1.0)
        print('lmotor2:', lmotor)
        print('rmotor2:', rmotor)
        print('tempo2:', time) 
    elif time>21.75 and time < 24.50:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(-1.0)
        print('lmotor3:', lmotor)
        print('rmotor3:', rmotor)
        print('tempo3:', time)
    elif time > 24.50 and time <= 34.0:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(1.0)
        print('lmotor4:', lmotor)
        print('rmotor4:', rmotor)
        print('tempo4:', time)
    elif time > 34.0 and time <= 36.75:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(-1.0)
        print('lmotor5:', lmotor)
        print('rmotor5:', rmotor)
        print('tempo5:', time)
    elif time > 36.75 and time <= 45.75:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(1.0)
        print('lmotor6:', lmotor)
        print('rmotor6:', rmotor)
        print('tempo6:', time)
    else:
        rmotor.setVelocity(0)
        lmotor.setVelocity(0)
pass     
    
    