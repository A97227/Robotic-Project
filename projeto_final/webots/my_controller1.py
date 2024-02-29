"""my_controller1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
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
#while robot.step(timestep) != -1:
    #tn = robot.getTime()
    #tm = tn - t
    
    #if tm < 2:
        #Linha reta
        #lmotor.setForce(0.5)
        #rmotor.setForce(0.5)
        #Rotação sentido horário
        #lmotor.setForce(0.5)
        #rmotor.setForce(-0.5)
        
    #else:
        #lmotor.setForce(0.0)
        #rmotor.setForce(0.0)
        
#pass

#Velocidade
#while robot.step(timestep) != -1:
    #tn = robot.getTime()
    #time = tn - t
    
    #if tn <= 2:
        #Linha reta
        #lmotor.setVelocity(5.0)
        #rmotor.setVelocity(5.0)
        #Rotação sentido horário
        #lmotor.setVelocity(5.0)
        #rmotor.setVelocity(-5.0)
        
    #else:
        #lmotor.setVelocity(0.0)
        #rmotor.setVelocity(0.0)
        
    #pass

#Posição
while robot.step(timestep) != -1:
    tn = robot.getTime()
    tm = tn - t
    #true_pos = (robot.getSelf().getPosition())
    
    if tm <= 2:
        #Linha reta
        #lmotor.setPosition(20.0)
        #rmotor.setPosition(20.0)
        #Rotação sentido horário
        lmotor.setPosition(5.0)
        rmotor.setPosition(-5.0)
        
    else:
        lmotor.setPosition(0.0)
        rmotor.setPosition(0.0)
        
    pass
    

