"""odometria controller."""
import math
import time
from controller import Keyboard
from controller import Robot, Supervisor
from controller import Node

DISTANCE_BETWEEN_WHEELS = 0.381
WHEEL_RADIUS = 0.097
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
lmotor = robot.getDevice('left wheel')
rmotor = robot.getDevice('right wheel')
lsensor = robot.getDevice('left wheel sensor')
rsensor = robot.getDevice('right wheel sensor')
lsensor.enable(1)
rsensor.enable(1)
lmotor.setPosition(float('inf'))
rmotor.setPosition(float('inf'))
lpos = lsensor.getValue()
rpos = rsensor.getValue()

#odometria
#robot.step(timestep)
#tn = robot.getTime()
#angle = 0.0
#x = 0.0
#y = 0.0
#t = 0.0
#lista = []

#while robot.step(timestep) != -1:
    #lpos = lsensor.getValue()
    #rpos = rsensor.getValue()
    #time = tn - t
    #true_pos = robot.getSelf().getPosition()
    
    #lvel = (lsensor.getValue() - lpos) / (time)
    #rvel = (rsensor.getValue() - rpos) / (time)
        
    #dx = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.cos(angle) 
    #dy = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.sin(angle) 
    #dangle = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rvel - lvel)
    #print('Velocidade esquerda:', lvel)
    #print('Velocidade direita:', rvel)    
    #x = x + dx * (time)
    #y = y + dy * (time)
    #angle = angle + dangle * (time)
    #print('Posição x: ' + str(x) + ',Posição y:' + str(y))
    #diferenca = math.sqrt((true_pos[0]-x) **2 + (true_pos[1]-y)**2)
    #print('Erro:', diferenca)
    #lista.append(diferenca)
    #print('Média:', (sum(lista)/len(lista)))
#pass

#Odometria comandos + erros
angle = 0.0
x = 0.0
y = 0.0
t = 0.0
lista = []
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
while robot.step(timestep) != -1:
    tn = robot.getTime()
    time = tn - t  
    lpos = lsensor.getValue()
    rpos = rsensor.getValue()
    vel = (2 * 3.14 * WHEEL_RADIUS) / (time)
    lvel = (lsensor.getValue() - lpos) / (time)
    rvel = (rsensor.getValue() - rpos) / (time)
    dx = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.cos(angle) 
    dy = 0.5 * WHEEL_RADIUS * (lvel+rvel) * math.sin(angle)
    dangle = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rvel - lvel)
    x = x + dx * (time)
    y = y + dy * (time)
    angle = angle + dangle * (time)
    key = keyboard.getKey()
    if key == Keyboard.UP:
        lmotor.setVelocity(vel)
        rmotor.setVelocity(vel)  
        true_pos = (robot.getSelf().getPosition())
        erro_total = math.sqrt((true_pos[0]-x) **2 + (true_pos[1]-y)**2)
        print('Erro da tecla up:' , erro_total)
    elif key == Keyboard.DOWN:
        lmotor.setVelocity(-vel)
        rmotor.setVelocity(-vel) 
        true_pos = (robot.getSelf().getPosition())
        erro_total = math.sqrt((true_pos[0]-x) **2 + (true_pos[1]-y)**2)
        print('Erro da tecla down: ', erro_total)
    else:
        lmotor.setVelocity(0.0)
        rmotor.setVelocity(0.0) 
pass
