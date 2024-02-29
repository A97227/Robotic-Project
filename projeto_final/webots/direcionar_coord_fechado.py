"""direcionar_coord_fechado controller."""
import math
import time
from controller import Keyboard
from controller import Robot, Supervisor
from controller import Node

#Em malha fechada
robot = Robot()
DISTANCE_BETWEEN_WHEELS = 0.381
WHEEL_RADIUS = 0.097
timestep = int(robot.getBasicTimeStep())
lmotor = robot.getDevice('left wheel')
rmotor = robot.getDevice('right wheel')
lsensor = robot.getDevice('left wheel sensor')
rsensor = robot.getDevice('right wheel sensor')
lsensor.enable(1)
rsensor.enable(1)
lmotor.setPosition(float('inf'))
rmotor.setPosition(float('inf'))
lmotor.setVelocity(-1.0)
rmotor.setVelocity(1.0)
tn = robot.getTime()
robot.step(timestep)
lpos = lsensor.getValue()
rpos = rsensor.getValue()
angle = 0.0
x = dx = 0.0
y = dy = 0.0
sec = 0
t = robot.getTime()

while robot.step(timestep) != -1:
    if sec == 0:
        if rpos<0.95 and lpos > -0.87:
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(-1.0)
            lpos = lsensor.getValue()
            rpos = rsensor.getValue()
            print('lpos:', lpos)
            print('rpos:', rpos) 
        else:
            sec = sec + 1
            rmotor.setVelocity(0)
            lmotor.setVelocity(0)
    elif sec == 1:
        if x < 2.30 and y < 1.35:
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(1.0)
            time = robot.getTime() - t
            lvel = (lsensor.getValue() - lpos) / (time)
            rvel = (rsensor.getValue() - rpos) / (time)
            dx = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.cos(angle)
            dy = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.sin(angle)
            dangle = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rvel - lvel)
            x = x + time * dx
            y = y + time * dy
            angle = angle + dangle * (time)
            lpos = lsensor.getValue()
            rpos = rsensor.getValue()
            print('lpos1:', lpos)
            print('rpos1:', rpos)
            print('x:', x)
            print('y:', y)
        else:
            rmotor.setVelocity(0)
            lmotor.setVelocity(0)
            y = 0     
    elif sec == 2:
        if rpos<25.5:
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(-1.0)
            lpos = lsensor.getValue()
            rpos = rsensor.getValue()
            print('lpos2:', lpos)
            print('rpos2:', rpos)
        else:
            sec = sec + 1
            rmotor.setVelocity(0)
            lmotor.setVelocity(0)
pass
            
    


