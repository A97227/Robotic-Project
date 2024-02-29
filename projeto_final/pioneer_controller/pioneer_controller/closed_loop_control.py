"""closed_loop_control module"""

from controller import Robot, Motor, PositionSensor
import math
# from math_utils import distance, angle

def distance(first, second):
    # Euclidean distance between two n-dimensional vectors *first* and *second* 
    assert(len(first) == len(second))
    sq_sum = 0.0
    for fval, sval in zip(first, second):
        sq_sum = sq_sum + (fval - sval)**2
        
    return math.sqrt(sq_sum)


def angle(vector, reference=[0, 0]):
    # get the angle of given *vector* with given reference frame (global by default)
    # useful to estimate required rotation to align robot with 2D coordinates
    assert(len(vector) == 2)
    dist = distance(vector, reference)
    if vector[0] >= 0:
        return math.asin((vector[1] - reference[1]) / dist)
    elif vector[1] >= 0:
        return math.acos((vector[0] - reference[0]) / dist)
    else:
        return 2.0 * math.pi - math.acos((vector[0] - reference[0]) / dist)


class DifferentialDriveOdometry(object):
    # Class that iteratively tracks the position + orientation shift of a 
    #    differential drive robot using velocity control commands
    # Implements forward kinematics and updates robot state on each call to update
    #    cf. https://ucr-robotics.readthedocs.io/en/latest/tbot/moRbt.html
    # @note requires wheel velocity for each update (measured from position sensor values)
  
    def __init__(self, wradius, wdistance, pos0=[0.0, 0.0, 0.0]):
        self._pos = pos0
        self._wradius = wradius
        self._wdistance = wdistance

    @property
    def position(self):
        return self._pos

    def update(self, wvel, tstep):
        # increment robot state with first derivatives (Euler's method) 
        assert(len(wvel) == 2)
        dvals = self.kinematics(wvel)
        self._pos[0] = self._pos[0] + dvals[0] * tstep
        self._pos[1] = self._pos[1] + dvals[1] * tstep
        self._pos[2] = self._pos[2] + dvals[2] * tstep

    def kinematics(self, wvel):
        # implementation of differential drive kinematics
        # cf. https://ucr-robotics.readthedocs.io/en/latest/tbot/moRbt.html
        dx = 0.5 * self._wradius * (wvel[0] + wvel[1]) * math.cos(self._pos[2])
        dy = 0.5 * self._wradius * (wvel[0] + wvel[1]) * math.sin(self._pos[2])
        dtheta = (self._wradius / self._wdistance) * (wvel[1] - wvel[0])

        return [dx, dy, dtheta]


def move(meters, odometry, robot, motors, encoders, wvel=math.pi, wpos0=[0,0],safety_threshold=0.5):
    # moves robot along it's frontal axis by *meters* (no rotation, translation only)
    # translation: forward if meters > 0, backward otherwise
    # closed-loop approach, requires *odometry* tracker and optionally wheel angular velocity *wvel*.
    
    assert(len(motors) == 2)
    assert(len(encoders) == 2)
    print('move ' + str(round(meters, 3)) + ' m')
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
        wpos = [encoders[0].getValue(), encoders[1].getValue()]
        diff = [wpos[0] - wpos0[0], wpos[1] - wpos0[1]]
        wvel = [diff[0] / timestep, diff[1] / timestep]
        wpos0 = wpos
        # update odometry
        odometry.update(wvel, timestep)
        # check if target has been reached
        if abs(distance(odometry.position, pos0)) > abs(meters):
            break
    # stop movement
    motors[0].setVelocity(0.0)
    motors[1].setVelocity(0.0)


def spin(ang, odometry, robot, motors, encoders, wvel=math.pi):
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


def reset(odometry, **kwargs):
    # reset robot to initial orientation (undo applied rotations)
    # closed-loop approach, requires *odometry* tracker (additional arguments forwarded to spin()) 
    
    ort = odometry.position[2]
    spin(-1.0 * ort, odometry, **kwargs)


def travel_to(target, odometry, tolerance=0.05, reverse=False, **kwargs):
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
    spin(ang, odometry, **kwargs)
    # move distance to target position
    # move(dist if not reverse else -1.0 * dist, odometry, robot=robot, motors=[lmotor, rmotor], encoders=[lencoder, rencoder], **kwargs)
    move(dist if not reverse else -1.0 * dist, odometry, **kwargs)
