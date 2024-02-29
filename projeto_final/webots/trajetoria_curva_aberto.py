"""trajetoria_curva_aberto controller."""
import math
import time
from controller import Keyboard
from controller import Robot, Supervisor
from controller import Node

#Figura 50
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
tn = robot.getTime()
angle = 30.0
raio = 2.0

while robot.step(timestep) != -1:
    time = robot.getTime() - tn
    if time <= 2.7:
        rmotor.setVelocity(1.0)
        lmotor.setVelocity(-1.0)
        print('lmotor1:', lmotor)
        print('rmotor1:', rmotor)
        print('tempo1:', time)
    elif time > 2.7 and time <= 5.1:
        variacao = time - 2.7
        if (variacao % 0.2) < timestep * 0.002:
            add = 1.0/2.0 #velocidade/raio
            rmotor.setVelocity(1.0 + add)
            lmotor.setVelocity(1.0 + add)
            print('lmotor2:', lmotor)
            print('rmotor2:', rmotor)
            print('tempo2:', time) 
    elif time > 5.1:
        rmotor.setVelocity(0.0)
        lmotor.setVelocity(0.0)
        print('lmotor3:', lmotor)
        print('rmotor3:', rmotor)
        print('tempo3:', time)
pass
