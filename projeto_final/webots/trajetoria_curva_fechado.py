"""trajetoria_curva_fechado controller."""
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
lmotor.setVelocity(1.0)
rmotor.setVelocity(1.0)
tn = robot.getTime()
robot.step(timestep)
lpos = lsensor.getValue()
rpos = rsensor.getValue()
angle_t = 30.0
raio = 2.0
x = 0
y = 0
angle = 0
linear_velocity = -1.0
time = robot.getTime() - tn
angle_rad = (angle_t * 3.14) / 180.0
total_time = 0.0
real_angle_rad = 0.0

while robot.step(timestep) != -1:
    velocidade_angular =  linear_velocity / raio
    lmotor.setVelocity(linear_velocity - velocidade_angular)
    rmotor.setVelocity(linear_velocity + velocidade_angular)
    real_angle_rad = ((rsensor.getValue() - lsensor.getValue()) / 2.0) / raio
    if abs(real_angle_rad) >= abs(angle_rad): #para os motores
        lmotor.setVelocity(0.0)
        rmotor.setVelocity(0.0)
        time = robot.getTime()
        lpos = lsensor.getValue()
        rpos = rsensor.getValue()
        robot.step(5) #esperar para não fazer divisão por zero
        lpos_1 = lsensor.getValue()
        rpos_1 = rsensor.getValue()
        lvel = (lpos_1 - lpos) / (5/1000)
        rvel = (rpos_1 - rpos) / (5/1000)
        dx = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.cos(angle)
        dy = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.sin(angle)
        dangle = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rvel - lvel)
        x1 = x + (robot.getTime() - time) * dx
        y1 = y + (robot.getTime() - time) * dy
        angle1 = angle + (robot.getTime() - time) * dangle
        print('Coordenadas:' + str(x1) + ',' + str(y1))
        #erro
        erro_x = abs(x - x1)
        erro_y = abs(y - y1)
        print('Erro de x:', erro_x)
        print('Erro de y:', erro_y)
     
    total_time = total_time + (timestep / 1000)
    print('Tempo Total:', total_time)
pass 
        

