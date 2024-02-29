"""sensor_feedback controller."""
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
lsensor = robot.getDevice('left wheel sensor')
rsensor = robot.getDevice('right wheel sensor')
lsensor.enable(1)
rsensor.enable(1)
lmotor.setPosition(float('inf'))
rmotor.setPosition(float('inf'))
lpos = lsensor.getValue()
rpos = rsensor.getValue()
t = 0.0

#sonar = []
#for idx in range(16):
    #sonar.append(robot.getDevice('so' + str(idx)))
    #sonar[-1].enable(1)
    
#leitura dos sensores e SONAR
#while robot.step(timestep) != -1:
    #tn = robot.getTime()
    #time = tn - t
    #lpos = lsensor.getValue()
    #rpos = rsensor.getValue()
    
    #sonar_values = []
    #for idx in range(16):
        #value = sonar[idx].getValue()
        #sonar_values.append(value)
    #print("Valores dos sonares: ", sonar_values)
    #print("Encoder esquerdo: ", lpos)
    #print("Encoder direito: ", rpos)
    
    #pass
#Deslocamento total
#deslocamento_total_l = 0
#deslocamento_total_r = 0
#lpos_0 = 0
#rpos_0 = 0
#while robot.step(timestep) != -1:
    #lpos = lsensor.getValue() 
    #rpos = rsensor.getValue()
    #calcular deslocamento
    #deslocamento_l = lpos - lpos_0
    #deslocamento_r = rpos - rpos_0
    #deslocamento_total_l = deslocamento_total_l + deslocamento_l
    #deslocamento_total_r = deslocamento_total_r + deslocamento_r
    #lpos_0 = lpos
    #rpos_0 = rpos
    #print("Deslocamento total da roda direita: ", deslocamento_total_r)
    #print("Deslocamento total da roda esquerda: ", deslocamento_total_l)
    
    #pass

def valores_sonar(sonar):
    table = sonar.getLookupTable()
    valores = sonar.getValue()
    distance = ((table[0] - table[3])/(table[1]-table[4]))*valores + table[3]
    return(distance)
    
sonar = []
for idx in range(16):
    sonar.append(robot.getDevice('so' + str(idx)))
    sonar[-1].enable(1)
while robot.step(timestep) != -1:
    for idx in range(16):
        if float(valores_sonar(sonar[idx])) < 0.5:
            lmotor.setVelocity(0.0)
            rmotor.setVelocity(0.0)
pass

#def valores_sonar(sonar):
    #for idx in range(16):
        #valores = sonar.getValue()
        #valor_maximo = max(valores)
        #distance = 5 - 5*(1-(valores/valor_maximo))
        #return (distance)
        #if ditance < 0.5:
            #lmotor.setVelocity(0.0)
            #rmotor.setVelocity(0.0)
            