#!/usr/bin/python
# -*- coding: utf-8 -*-
import random
from visual import vector
from math import sqrt,pi
'''
Created on 26/07/2014

@author: Francisco Dominguez
'''
class Robot(object):
    def __init__(self,s,k,m):
        '''
        Abstract Robot constructor
        '''
        self.sensor=s
        self.kinematic=k
        self.map=m
        self.pos=vector(0.0,0.0)
        self.orientation=0.0
        self.v=vector(0.0,0.0,0.0)
        self.w=vector(0.0,0.0,0.0)
    def __str__(self):
        return '[x=%.6s y=%.6s o=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
    #Clone methods
    def copyRandom(self):
        ''' this is a very shallow copy '''
        r=Robot(self.sensor,self.kinematic,self.map)
        (width,height)=self.map.dim
        originX=self.map.origin.x
        originY=self.map.origin.y
        r.pos=vector(random.random()*width-originX,random.random()*height-originY)
        r.orientation = self.orientation #float(random.random() * 2.0 * pi)
        return r       
    def copy(self):
        ''' this is a very shallow copy '''
        r=Robot(self.sensor,self.kinematic,self.map)
        self.pos=vector(self.pos)
        self.orientation = float(self.orientation)        
        return r       
    #Access methods
    def getTranslationalVelocity(self):
        return self.v
    def getRotationalVelocity(self):
        return self.w
    def getPos(self):
        return self.pos
    def getOrientation(self):
        return self.orientation
    def set(self, new_x, new_y, new_orientation):
        self.pos=vector(float(new_x),float(new_y))
        self.orientation = float(new_orientation)        
    def setSensor(self,s):
        self.sensor=s 
    def setKinematic(self,k):
        self.kinematic=k
    def setMap(self,m):
        self.map=m      
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sensor.setNoise(new_s_noise);   
    #Sensor methods
    def sense(self):
        return self.sensor.sense(self)      
    def measurement_prob(self, measurement):
        return self.sensor.measurement_prob(self, measurement)
    #Motion methods
    def move(self, turn, forward):
        (x,y,o)=self.kinematic.move(self,turn,forward)
        #self.set(x, y, o)
        r=self.copy()
        r.set(x, y, o)
        return r 
        
    # --------
    # check: 
    #    checks of the robot pose collides with an obstacle, or
    # is too far outside the plane
    def check_collision(self, grid):
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 1:
                    dist = sqrt((self.x - float(i)) ** 2 + 
                                (self.y - float(j)) ** 2)
                    if dist < 0.5:
                        self.num_collisions += 1
                        return False
        return True
        
    def check_goal(self, goal, threshold = 1.0):
        dist =  sqrt((float(goal[0]) - self.x) ** 2 + (float(goal[1]) - self.y) ** 2)
        return dist < threshold
    
