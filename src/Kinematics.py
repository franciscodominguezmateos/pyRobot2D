#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import pi,tan,sin,cos
import random
'''
Created on 27/07/2014

@author: Francisco Dominguez
'''
class Kinematic(object):
    '''
    classdocs
    '''
    def __init__(self,params):
        '''
        Constructor
        '''
class VelocityMotionModel(Kinematic):
    def __init__(self):
        self.alpha1=1.0
        self.alpha2=1.0
        self.alpha3=1.0
        self.alpha4=1.0
        self.alpha5=1.0
        self.alpha6=1.0
    def move(self,robot,dt=0.1):
        x=robot.getPos().x
        y=robot.getPos().y
        theta=robot.getOrientation()
        v=robot.getTranslationalVelocity()
        w=robot.getRotationalVelocity()
        a1,a2,a3,a4,a5,a6=self.alpha1,self.alpha2,self.alpha3,self.alpha4,self.alpha5,self.alpha6
        vt=v+random.gauss(0,a1*v^2+a2*w^2)
        wt=w+random.gauss(0,a3*v**2+a4*w**2)
        gammat=random.gauss(0,a5*v**2+a6*w**2)
        xp=x-vt/wt*sin(theta)+vt/wt*cos(theta+wt*dt)
        yp=y+vt/wt*cos(theta)-vt/wt*sin(theta+wt*dt)
        thetap=theta+wt*dt+gammat*dt
        return (xp,yp,thetap)
    
class KinematicDifferential(Kinematic):
    '''
        Two differential wheels
    '''
    def __init__(self):
        '''
        Constructor Kinematic differential two wheels
        '''
        self.b=10 #distance between the two wheels of differential-drive robot
        self.r=12 #wheel radious
        
class KinematicBicycle(Kinematic):
    '''
    Instantaneous Center of Rotation (ICR) motion model
    It is the same por bicycle, cars, etc.
    '''
    def __init__(self):
        '''
        Constructor Kinematic Bicycle ICR model
        '''
        self.steering_noise=2*pi/60.0
        self.distance_noise=100.0
        self.length = 150.0 # distance from rear to front wheels
    # --------
    # move: 
    #    robot    = motion model to apply
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative
    def move(self, robot, steering, distance, 
             tolerance = 0.001, max_steering_angle = pi / 4.0):
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0
        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        # Execute motion
        turn = tan(steering2) * distance2 / self.length
        if abs(turn) < tolerance:
            # approximate by straight line motion
            x = robot.pos.x + (distance2 * cos(robot.orientation))
            y = robot.pos.y + (distance2 * sin(robot.orientation))
            orientation = (robot.orientation + turn) % (2.0 * pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = robot.pos.x - (sin(robot.orientation) * radius)
            cy = robot.pos.y + (cos(robot.orientation) * radius)
            orientation = (robot.orientation + turn) % (2.0 * pi)
            x = cx + (sin(orientation) * radius)
            y = cy - (cos(orientation) * radius)
        return (x,y,orientation)
        
class KinematicTurtle(Kinematic):
    def __init__(self):
        self.turn_noise=2*pi/360.0
        self.forward_noise=10.0
    # --------
    # move: 
    #    robot    = motion model to apply
    #    turn = angle turn the robot
    #    forward = total distance driven, most be non-negative
    def move(self, robot, turn, forward):
        if forward < 0:
            raise ValueError, "Robot can't move backwards"        
        # turn, and add randomness to the turning command
        orientation = (robot.orientation + float(turn) + random.gauss(0.0, self.turn_noise)) % (2*pi)       
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = robot.pos.x + (cos(orientation) * dist)
        y = robot.pos.y + (sin(orientation) * dist)
        return (x,y,orientation)
