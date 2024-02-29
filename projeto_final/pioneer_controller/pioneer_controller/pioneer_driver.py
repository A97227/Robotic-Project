from termios import TAB2
import time
from random import randint
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np
import math
import random
from rclpy.exceptions import ParameterNotDeclaredException
from sensor_msgs.msg import Range, LaserScan 
from pioneer_controller.closed_loop_control import DifferentialDriveOdometry, distance, angle, move, spin, travel_to
from pioneer_interfaces.srv import SetValue, SetTravel


# from pioneer_controller.closed_loop_control import DifferentialDriveOdometry, distance, angle
# from pioneer_interfaces.srv import SetValue, Travel

DISTANCE_BETWEEN_WHEELS = 0.381
WHEEL_RADIUS = 0.097
NUM_SONAR = 16
Max_sensor_value = 1024

# @note in order to interface with webots_ros2_driver package (cf. https://index.ros.org/p/webots_ros2_driver/)
#       driver classes must implement init() and step() members
class DifferentialDriveController:
    def init(self, webots_node, properties):
        # entry point to the Webots API (cf. https://cyberbotics.com/doc/reference/nodes-and-api-functions)
    
        self.__robot = webots_node.robot
        
        # initialize motor objects
        self.__left_motor = self.__robot.getDevice('left wheel')
        self.__right_motor = self.__robot.getDevice('right wheel')

        # enable velocity control
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0) 

        # create sensor interface 
        # @note sensores have to be enable before reading 
        
        self.__left_encoder = self.__robot.getDevice('left wheel sensor')
        self.__right_encoder = self.__robot.getDevice('right wheel sensor')
        self.__left_encoder.enable(1)
        self.__right_encoder.enable(1)

        
        # target twist to apply to the robot
        self.__target_twist = Twist()


        # ROS-specific
        # initializes 1) ROS Python modules, 2) initializes a 'pioneer_driver' node and creates a new subcription to the 'cmd_vel' topic
        rclpy.init(args = None)
        self.__node = rclpy.create_node('pioneer_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        
        
        # exercicio 2.B1
        # cria o publisher do encoder
        self._left_encoder_publisher = self.__node.create_publisher(Float64, 'left_wheel_position', 10)
        self._right_encoder_publisher = self.__node.create_publisher(Float64, 'right_wheel_position', 10)

        # exercicio 2.B2  
        # criar o publisher da velocidade e de odometria
        self._left_velocity_publisher = self.__node.create_publisher(Float64, 'left_wheel_velocity', 10)
        self._right_velocity_publisher = self.__node.create_publisher(Float64, 'right_wheel_velocity', 10)

        self.__odometry_publisher = self.__node.create_publisher(Twist, 'odometry', 10)

        # variáveis para armazenar as velocidades 

        self._left_wheel_velocity = 0.0
        self._right_wheel_velocity = 0.0

        # variáveis para armazenar as últimas posições conhecidas
        self._last_left_wheel_position = 0.0
        self._last_right_wheel_position = 0.0

        # último tempo 
        self.__last_time = self.__node.get_clock().now().seconds_nanoseconds()

        # calculo da odometria diferencial do robot com base nas velocidades
        self.__odometry = DifferentialDriveOdometry(WHEEL_RADIUS, DISTANCE_BETWEEN_WHEELS)
        self.__safety_threshold=0.5

        # exercicio 2.B3
        # criação um serviço: move e spin

        self.__node.create_service(SetValue, 'move', self.__move_callback)
        self.__node.create_service(SetValue, 'spin', self.__spin_callback)

        # exercício 2.B4
        # fazer load e ler valor do parâmetro, respetivamente
        self.__node.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.__node.declare_parameter('distance_between_wheels', DISTANCE_BETWEEN_WHEELS)

        self.WHEEL_RADIUS = self.__node.get_parameter("wheel_radius").get_parameter_value().double_value
        self.DISTANCE_BETWEEN_WHEELS = self.__node.get_parameter("distance_between_wheels").get_parameter_value().double_value
   
        # exercicio 2.B5
        # mover o robot para as coordenadas-alvo
        self.__node.create_service(SetTravel, 'travel_to', self.__travel_to_callback)

        
        # Indica que nenhum obstáculo foi detetado
        self._obstacle_detected = False

        # exercicio 2.B6
        self.sonar = list()
        for idx in range(NUM_SONAR):
            self.sonar.append(self.__robot.getDevice('so' + str(idx)))
            self.sonar[-1].enable(1)


        self.__sonar_dist = dict()
        self.__sonar_sub = []
        for idx in range(NUM_SONAR):
            topic = '/Pioneer_3_DX/so' + str(idx)
            self.__sonar_dist[topic] = 5.0
            self.__sonar_sub.append(self.__node.create_subscription(Range, topic, self.__sonar_callback, 10))


# C1 -> Inicialização
        self.LIDAR=self.__robot.getDevice("Hokuyo URG-04LX-UG01")
        self.__lidar_sub = self.__node.create_subscription(LaserScan, "Pioneer_3_DX/Hokuyo_URG_04LX_UG01",self.__laser_callback, 10)
        self.lidar_ranges = []  #armazena leituras do LIDAR
        #armazenar os valores min e max dos angulos de leitura do LIDAR
        self.lidar_angulomin = 0
        self.lidar_angulomax = 1
        self.obstacle = False  #flag que indica se um obstáculo foi detetado

#Exercicio B6 /C1
        self.issonar=0
        self.estado="stopped"
        self.request1=0
        self.request2=0
    def __cmd_vel_callback(self, twist):
        # Atualiza o alvo twist em cada publicaçãono topico '/cmd_vel'
        self.__target_twist = twist


#func aux para B3/B5
    def save_var(self,requests1,requests2,type):
        self.estado=type
        self.request1=requests1
        self.request2=requests2
        return


    # exercicio B3--> Implementar o movimento para frente/trás
    def __move_callback(self, request, response):
        self.save_var(request.value,0,'move')

        return response

    #exercise B3) --> IMPLEMENT 'SPIN' SERVICE FOR AXIAL ROTATION 
    def __spin_callback(self, request, response):
        self.save_var(request.value,0,'spin')
        return response

     # EXERCICIO B5
    def __travel_to_callback(self, request, response):
        self.save_var(request.value_x,request.value_y,'travel_to')
        return response
    

    # EXERCICIO B6
    def __sonar_callback(self, range):
        label =f"Pioneer_3_DX/{range.header.frame_id}"

        self.__sonar_dist[label]=range.range


    def check_sonar(self,safety_threshold):
        for idx in range(NUM_SONAR):
            valor=self.sonar[idx].getValue()
            distancia=5-5*(1-(valor/Max_sensor_value))

            if distancia!=0.0:
                if distancia < safety_threshold:
                    self.__node.get_logger().info(f"Stopping due to finding an obstacle at: so{idx}")
                    self.__left_motor.setVelocity(0)
                    self.__right_motor.setVelocity(0)
                    self._obstacle_detected = True
                    return False
        return True
    
     # exercicio C1
    def __laser_callback(self, msg):  #atualiza as variáveis relacionadas ao Lidar com os dados recebidos.
        # Limpa a lista de leituras anteriores do Lidar
        self.lidar_ranges = []
        for i in msg.ranges:
            # Adiciona cada leitura à lista lidar_ranges
            self.lidar_ranges.append(i)
        # Atualiza as variáveis que armazenam os ângulos mínimo e máximo de leitura do Lidar
        self.lidar_angulomin = msg.angle_min
        self.lidar_angulomax = msg.angle_max
        # Atualiza a variável que armazena o incremento angular entre as leituras
        self.lidar_ang_inc = msg.angle_increment   
   
    
   


    def check__lidar(self, threshold=0.8, gap=math.pi / 8, wvel=math.pi, rvel=math.pi / 6):
        #comprimento da lista de leituras do Lidar
        Values=self.LIDAR.getRangeImage()
        comp = len(self.lidar_ranges)

        #indice minimo da faixa à frente
        min = round(comp / 2 - (comp * gap / abs(self.lidar_angulomax - self.lidar_angulomin)))
        #indice maximo
        max = round(comp / 2 + (comp * gap / abs(self.lidar_angulomax - self.lidar_angulomin)))

        #leituras do Lidar na faixa à frente
        self.forward_range = Values[min:max:3]
        m = 5
        lista = [1, 1]
        self.obstacle = False
        if not self.obstacle:
            self.dir = lista[round(randint(0, 1))]
        for i in self.forward_range:
            
            if i < m:
                m = i
            if i < threshold:  #leitura abaixo do threshold -> obstaculo
                self.__left_motor.setPosition(math.inf)
                self.__right_motor.setPosition(math.inf)
                self.__left_motor.setVelocity(-rvel if self.dir > 0 else rvel)
                self.__right_motor.setVelocity(rvel if self.dir > 0 else -rvel)
                self.obstacle=True
            elif not self.obstacle:
                self.obstacle = False
                p = 1
                self.__left_motor.setPosition(math.inf)
                self.__right_motor.setPosition(math.inf)
                self.__left_motor.setVelocity(wvel * p)
                self.__right_motor.setVelocity(wvel * p)

        if not self.obstacle:
            print("No obstacle detected")
        else:
            print("Obstacle detected")


    def step(self):
        # spin ROS nodes/topics
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # EXERCICIO 2B1
        self.publish_wheel_sensors()

        #exercicio 2b2
        if self.estado=='move':
            self.move(self.request1, self.__odometry, self.__robot, [self.__left_motor,self.__right_motor],
             [self.__left_encoder,self.__right_encoder],math.pi,self.__safety_threshold)
            self.__left_motor.setVelocity(0)
            self.__right_motor.setVelocity(0)
            self.estado='Stopped'
        elif self.estado=='spin':
            self.spin(self.request1, self.__odometry, self.__robot, [self.__left_motor,self.__right_motor],
             [self.__left_encoder,self.__right_encoder],math.pi)
            self.estado='Stopped'
        elif self.estado=='travel_to':
            velocity = math.pi
            travel_to((self.request1,self.request2),
            self.__odometry,robot= self.__robot, motors= [self.__left_motor,self.__right_motor],
                  encoders = [self.__left_encoder,self.__right_encoder],wvel=velocity)
            self.estado='Stopped'
        time = self.__node.get_clock().now().seconds_nanoseconds()
       

        elapsed = time[0] - self.__last_time[0] + (1e-9) * (time[1] - self.__last_time[1])
        self.publish_wheel_velocity(elapsed)
        self.publish_odometry(elapsed)

        # backup current time stamp and wheel position for next update
        self.__last_time = time
        self._last_left_wheel_position = self.__left_encoder.getValue()
        self._last_right_wheel_position = self.__right_encoder.getValue()
        self.__last_time = self.__node.get_clock().now().seconds_nanoseconds()


        

        #exercicio 2.B1

                
        # get linear and angular speeds to apply to the robot (COM)
        # @note 'x' axis: forward movement and 'z' axis turn around its COM
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # convert to velocity commands and apply them to the motors
        command_motor_left = (forward_speed - angular_speed * 0.5 * DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * 0.5 * DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        # exercicio 2.B3

    def move(self,meters, odometry, robot, motors, encoders, wvel=math.pi, wpos0=[0,0],safety_threshold=0.5):
        # moves robot along it's frontal axis by *meters* (no rotation, translation only)
        # translation: forward if meters > 0, backward otherwise
        # closed-loop approach, requires *odometry* tracker and optionally wheel angular velocity *wvel*.
        assert(len(motors) == 2)
        assert(len(encoders) == 2)
        if not meters:
            return
    # get current position (coordinates)
        pos0 = odometry.position[:]
    # read and store initial encoder values
    # @note enconders can't be sampled until robot.step() is called
        timestep = int(robot.getBasicTimeStep())
        robot.step(timestep)
        wpos0 = [encoders[0].getValue(), encoders[1].getValue()]
    # enable velocity control
        motors[0].setPosition(math.inf)
        motors[1].setPosition(math.inf)
        motors[0].setVelocity(wvel if meters > 0 else -wvel)
        motors[1].setVelocity(wvel if meters > 0 else -wvel)
    # move robot / step simulation
        while robot.step(timestep) != -1:

            
        # estimate wheel velocities from encoders
            #B6 /C1
            if self.issonar: 
                if not self.check_sonar(self.__safety_threshold):
                    break
            else:
                self.check__lidar()

                
            wpos = [encoders[0].getValue(), encoders[1].getValue()]
            diff = [wpos[0] - wpos0[0], wpos[1] - wpos0[1]]
            wvel = [diff[0] / timestep, diff[1] / timestep]
            wpos0 = wpos
        # update odometry
            odometry.update(wvel, timestep)
        # check if target has been reached
            if abs(distance(odometry.position, pos0)) > abs(meters):
                break
 
    # exercicio 2.B1
    def publish_wheel_sensors(self):
        #read and publ. wheel encoders
        left_encoder_msg = Float64()
        left_encoder_msg.data = self.__left_encoder.getValue()
        self._left_encoder_publisher.publish(left_encoder_msg)
        
        right_encoder_msg = Float64()
        right_encoder_msg.data = self.__right_encoder.getValue()
        self._right_encoder_publisher.publish(right_encoder_msg)
        
    #exercicio 2.B2
    def publish_wheel_velocity(self, elapsed):
        # update velocity values
        self._left_wheel_velocity = (self.__left_encoder.getValue() - self._last_left_wheel_position) / elapsed
        self._right_wheel_velocity = (self.__right_encoder.getValue() - self._last_right_wheel_position) / elapsed
        
        #publish velocity estimate
        left_velocity_msg = Float64()
        left_velocity_msg.data = self._left_wheel_velocity 
        self._left_velocity_publisher.publish(left_velocity_msg)
        
        right_velocity_msg = Float64()
        right_velocity_msg.data = self._right_wheel_velocity 
        self._right_velocity_publisher.publish(right_velocity_msg)

    def publish_odometry(self, elapsed):
        self.__odometry.update((self._left_wheel_velocity, self._right_wheel_velocity), elapsed)

        odometry_msg = Twist()
        odometry_msg.linear.x = self.__odometry.position[0]
        odometry_msg.linear.y = self.__odometry.position[1]
        odometry_msg.linear.z = self.__odometry.position[2]

        self.__odometry_publisher.publish(odometry_msg)



        #funções aux

    def spin(self,ang, odometry, robot, motors, encoders, wvel=math.pi):
    # spins robot *ang* radians along it central axis (no translation, rotation only)
    # useful to change robot orientation for forward movement
    # rotation: counterclockwise if angle > 0, clockwise otherwise
    # closed-loop approach, requires *odometry* tracker and optionally wheel angular velocity *wvel*.
    
        assert(len(motors) == 2)
        assert(len(encoders) == 2)
        print('spin ' + str(round(ang, 3)) + ' rad')
        if not ang:
            return
        # get current orientation 
        angle0 = odometry.position[2]
        # read and store initial encoder values
        # @note enconders can't be sampled until robot.step() is called
        timestep = int(robot.getBasicTimeStep())
        robot.step(timestep)
        wpos0 = [encoders[0].getValue(), encoders[1].getValue()]
        # enable velocity control
        motors[0].setPosition(math.inf)
        motors[1].setPosition(math.inf)
        motors[0].setVelocity(-wvel if ang > 0 else wvel)
        motors[1].setVelocity(wvel if ang > 0 else -wvel)
        # move robot / step simulation
        while robot.step(timestep) != -1:
            # estimate wheel velocities from encoders
            wpos = [encoders[0].getValue(), encoders[1].getValue()]
            diff = [wpos[0] - wpos0[0], wpos[1] - wpos0[1]]
            wvel = [diff[0] / timestep, diff[1] / timestep]
            wpos0 = wpos
            # update odometry
            odometry.update(wvel, timestep)
            # check if target has been reached
            if abs(odometry.position[2] - angle0) > abs(ang):
                break
        # stop movement
        motors[0].setVelocity(0.0)
        motors[1].setVelocity(0.0)

    def travel_to(self,target, odometry, tolerance=0.05, reverse=False, **kwargs):
    # moves robot towards given *target* coordinates wrapping around spin() and move()
    # movement is front-first by default, optionally reverse movement can be enforced with *reverse*
    # closed-loop approach, requires *odometry* tracker (additional arguments forwarded to spin() and move()) 
    
        # get current position
        pos = odometry.position[:2]
        # compute distance to target
        dist = distance(pos, target)
        if dist < tolerance:
            return
        # offset target (translate coordinates to robot frame)
        # cf. vector subtraction / trignonometric approach
        target = [target[0] - pos[0], target[1] - pos[1]]
        # estimate required spin (offset by current robot orientation)
        ang = angle(target) - odometry.position[2]
        if reverse:
            ang = ang - math.pi
        # find shortest movement (clockwise / counter-clockwise)
        # optional, results in a shorter movement duration
        if ang > math.pi:
            ang = ang - (2.0 * math.pi)
        if ang < -1.0 * math.pi:
            ang = ang + (2.0 * math.pi)
        # set robot orientation (towards target position)
        # spin(ang, odometry, robot=robot, motors=[lmotor, rmotor], encoders=[lencoder, rencoder], **kwargs)
        self.spin(ang, odometry, **kwargs)
        # move distance to target position
        # move(dist if not reverse else -1.0 * dist, odometry, robot=robot, motors=[lmotor, rmotor], encoders=[lencoder, rencoder], **kwargs)
        self.move(dist if not reverse else -1.0 * dist, odometry, **kwargs)