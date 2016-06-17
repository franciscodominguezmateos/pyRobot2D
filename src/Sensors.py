#!/usr/bin/python
# -*- coding: utf-8 -*-
from math import sqrt,exp,pi,sin,cos,atan2,abs
from visual import vector
import random
from Maps import MapLandmarks,MapContinous
from TinyGeometry.Line import Line
'''
Created on 26/07/2014

@author: Francisco Dominguez
'''
#just define a interface for sensors
class Sensor:    
    def sense(self,robot):
        #return data measurement from robot given this sensor
        Z = []
        return Z
    def measurement_prob(self,robot, measurement):        
        # calculates how likely a measurement should be        
        # given the robot prior associated data   
        prob = 1.0;
        return prob
class SensorRange(Sensor):
    def inverse_sense(self,mij,robot,z):
        l0=0
        locc=0
        lfree=0
        zMax=0
        zk=0
        alpha=0
        beta=0
        thetajsens=0
        (x,y)=robot.getPose()
        theta=robot.getOrientation()
        (xi,yi)=mij
        dx=xi-x
        dy=yi-y
        r=sqrt(dx*dx+dy*dy)
        phi=atan2(dy,dx)-theta
        #k=argmin(abs(phi-thetajsens))
        k=0
        if r>min(zMax,zk+alpha/2) or abs(phi-thetajsens)>beta/2:
            return l0
        if z[k]<zMax and abs(r-zMax)<alpha/2:
            return locc
        if r<z[k]:
            return lfree
    def sense(self,robot):
        pass
    def measurment_prob(self,robot,measurement):
        pass
class SensorLandmarks(Sensor):
    def __init__(self):
        self.landmarks=MapLandmarks()
        #Noise of all landmark sensors
        self.sense_noise  = 0.0;
    def deepcopy(self):
        s=SensorLandmarks()
        s.sense_noise=self.sense_noise
        return s
    #Landmarks access methods
    def appendLandmarkPos(self,p):
        self.landmarks.append(p)
    def setLandmarks(self,lm):
        self.landmarks=lm
    def getLandmarks(self):
        return self.landmarks
    #Noise access methods
    def setNoise(self, new_s_noise):
        self.sense_noise   = float(new_s_noise);
    def getNoise(self):
        return self.sense_noise
    def sense(self,robot):
        Z = []
        for i in range(len(self.landmarks)):
            dist = sqrt((robot.x - self.landmarks.get(i)[0]) ** 2 + (robot.y - self.landmarks.get(i)[1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    def measurement_prob(self, robot, measurement):        
        # calculates how likely a measurement should be    
        # given the robot prior associated data   
        prob = 1.0;
        for i in range(len(self.landmarks)):
            dist = sqrt((robot.x - self.landmarks.get(i)[0]) ** 2 + (robot.y - self.landmarks.get(i)[1]) ** 2)
            g=self.Gaussian(dist, self.sense_noise, measurement[i])
            prob *= g
        return prob
    
class SensorLIDAR(Sensor):
    def __init__(self,Nmeasures=36):
        '''
        Constructor SensorLIDAR
        Is a 360º laser range
        0º is just right and turn CCW
        '''
        self.map=MapContinous()
        self.Nmeasures=Nmeasures
        self.DegreeMeasure=360/self.Nmeasures
        self.sense_noise=100.0
    def getLine(self,i,robot):
        deg=(i*self.DegreeMeasure+robot.orientation*180.0/pi) % 360
        rad=deg*pi/180.0
        v=vector(cos(rad),sin(rad))
        p0=robot.getPos()
        p1=p0+v
        return Line(p0,p1)
    def getDist(self,i,robot):
        line=self.getLine(i,robot)
        intersectPoint=robot.map.nearestIntersectionPoint(line)
        dist=(robot.pos-intersectPoint).mag
        return dist,intersectPoint
    def sense(self,robot):
        Z=[]
        for i in range(self.Nmeasures):
            dist,p=self.getDist(i,robot)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    def sensePoints(self,robot):
        Z=[]
        for i in range(self.Nmeasures):
            dist,p=self.getDist(i,robot)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append((dist,p))
        return Z
    def measurement_prob(self, robot, measurement):        
        # calculates how likely a measurement should be    
        # given the robot prior associated data   
        prob = 1.0;
        for i in range(self.Nmeasures):
            dist,p =self.getDist(i,robot)
            g=self.Gaussian(dist, self.sense_noise, measurement[i])
            prob *= g
        return prob
    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
class SensorGPS(Sensor):
    def __init__(self):
        self.measurement_noise=0.0
    def deepcopy(self):
        s=SensorGPS()
        s.measurement_noise=self.measurement_noise
        return s
    #Noise access methods
    def setNoise(self, new_s_noise):
        self.measurement_noise   = float(new_s_noise);
    def getNoise(self):
        return self.measurement_noise
    #return a noised GPS 2D position
    def sense(self,robot):
        return [random.gauss(robot.x, self.measurement_noise),
                random.gauss(robot.y, self.measurement_noise)]
    # --------
    # measurement_prob
    #    computes the probability of a measurement
    # 
    def measurement_prob(self, robot, measurement):
        # compute errors
        error_x = measurement[0] - robot.x
        error_y = measurement[1] - robot.y
        # calculate Gaussian
        error = exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))
        error *= exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))
        return error
class SensorIMU(Sensor):
    def __init__(self):
        pass

