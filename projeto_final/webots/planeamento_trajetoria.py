"""planeamento_trajetoria controller."""
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
t = 0
while robot.step(timestep) != -1:
    if sec == 0:
        if rpos<2.60:
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
        if y < 2: #andar 2 metros
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(1.0)
            time = robot.getTime() - t
            lvel = (lsensor.getValue() - lpos) / (time)
            rvel = (rsensor.getValue() - rpos) / (time)
            dy = 0.5 * WHEEL_RADIUS * (lvel+rvel) 
            y = y + dy * (time)
            lpos = lsensor.getValue()
            rpos = rsensor.getValue()
            print('lpos1:', lpos)
            print('rpos1:', rpos)
            print('y:', y)
        else:
            sec = sec + 1
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
    elif sec == 3:
        if x < 1: #andar 1 metro
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(1.0)
            time = robot.getTime() - t
            lvel = (lsensor.getValue() - lpos) / (time)
            rvel = (rsensor.getValue() - rpos) / (time)
            dx = 0.5 * WHEEL_RADIUS * (lvel+rvel) 
            x = x + time * dx
            lpos = lsensor.getValue()
            rpos = rsensor.getValue() 
            print('lpos3:', lpos)
            print('rpos3:', rpos)
            print('x:', x)  
        else:
            sec = sec + 1
            rmotor.setVelocity(0)
            lmotor.setVelocity(0)
    elif sec == 4:
        if lpos > 24.2:
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(-1.0)
            lpos = lsensor.getValue()
            rpos = rsensor.getValue()
            print('lpos4:', lpos)
            print('rpos4:', rpos)
        else:
            sec = sec + 1
            rmotor.setVelocity(0)
            lmotor.setVelocity(0)
    elif sec == 5:
        if y<1: #andar 1 metros
            rmotor.setVelocity(1.0)
            lmotor.setVelocity(1.0)
            time = robot.getTime() - t
            lvel = (lsensor.getValue() - lpos) / (time)
            rvel = (rsensor.getValue() - rpos) / (time)
            dy = 0.5 * WHEEL_RADIUS * (lvel+rvel) 
            y = y + time* dy
            lpos = lsensor.getValue()
            rpos = rsensor.getValue()
            print('lpos5:', lpos)
            print('rpos5:', rpos)
        else:
            sec = sec + 1
            rmotor.setVelocity(0)
            lmotor.setVelocity(0)
pass
            
    
    
    
    