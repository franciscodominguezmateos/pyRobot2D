#!/usr/bin/python
# -*- coding: utf-8 -*-
from visual import vector
import numpy as np
from TinyGeometry.LineExceptions import NoIntersectLines,PointNotIntersectSegments

'''
Created on 26/07/2014

@author: Francisco Dominguez
'''

class Map(object):
    '''
    classdocs
    '''
    def __init__(self,params):
        '''
        Constructor
        '''
        self.size=None
        self.origin=vector(0,0)
        self.dim=(100,100)
    def getSize(self):
        return self.size
    def intersectionPoints(self,line):
        #return a intersection points from line in this map
        pass
    def nearestIntersectionPoint(self,line):
        #return the nearest intersected point 
        pass
    def isOutOfBound(self,point):
        pass
    
class Landmark(object):
    def __init__(self):
        self.pos=vector(0,0,0)
        self.radious=10
    def intersection(self,line):
        '''
        return intersection point from a line and a sphere
        '''
class MapLandmarks(Map):
    def __init__(self):
        ''' 
        Constructor MapLandmarks
        '''        
        self.landmarks=[]
    def getLandmarks(self):
        return self.lanmarks
    def setLandmarks(self,landmarks):
        self.landmarks=landmarks
    def append(self,l):
        self.landmarks.apend(l)
    def get(self,i):
        return self.landmarks[i]
    def intersectionPoints(self,line):
        #return all intersection points from line in this map
        points=[]
        for l in self.landmarks:
            p=l.intersection(line)
            points.append(p)
        return points
    def nearestIntersectionPoint(self,line):
        minPoint=vector(10e100,10e100)
        minD=10e100
        for l in self.landmarks:
            p=l.intersection(line)
            d=(minPoint-p).mag()
            if d<minD:
                minD=d
                minPoint=p
        return minPoint
                
class MapGrid(Map):
    def __init__(self,params):
        '''
        Constructor MapGrid
        '''
        self.size=(400,400)
        self.grid=np.zeros(self.size,'f')
        self.sizeCell=50 #50mm each cell (20m,20m)
        self.dim=(self.size[0]*self.sizeCell,self.size[1]*self.sizeCell) #(20000mm,20000mm)=(20m,20m)
        self.origin=vector(0,0) #in mm
        self.occupacyThreshold=0.6 # if value in a cell is bellow this threshold the cell is empty
    def getCellFromMM(self,point):
        p=point+self.origin
        i=p.x % self.sizeCell
        j=p.y % self.sizeCell
        return self.grid[i,j]
    def isOutOfBound(self,point):
        p=point+self.origin
        if p.x<0:
            return True
        if p.y<0:
            return True
        if p.x>self.dim[0]:
            return True
        if p.y>self.dim[1]:
            return True
        return False
    def getLineCells(self,line):
        #return all cells this line pass through them
        pCells=[]
        pOrigin=line.getP()
        if self.isOutOfBound(pOrigin):
            return pCells
        t=0
        v=line.getV()
        p=pOrigin+v*t
        while not self.isOutOfBound(p):
            pCells.append(p)
            t+=self.sizeCell
            p+=v*t
        return pCells
    def intersectionPoints(self,line):
        #return all intersection points from line in this map
        points=[]
        for p in self.getLineCells(line):
            if self.getCellFromMM(p)>=self.occupacyThreshold:
                points.append(p)
        return points
    def nearestIntersectionPoint(self,line):
        #return all cells this line pass through them
        pOrigin=line.getP()
        if self.isOutOfBound(pOrigin):
            raise ValueError("Line origin=",pOrigin," is out of bound map")
        t=0
        v=line.getV()
        p=pOrigin+v*t
        while not self.isOutOfBound(p):
            t+=self.sizeCell
            p+=v*t
            if self.getCellFromMM(p)>self.occupacyThreshold:
                return p
        raise ValueError("Not nearest Intersection Point found in MapGrid")
    
class MapTopological(Map):
    def __init__(self,params):
        '''
        Constructor Maptopological
        '''
    def intersectionPoints(self,line):
        pass
    def nearestIntersectionPoint(self,line):
        #return the nearest intersected point 
        pass
    
class MapContinous(Map):
    def __init__(self,vOrigin=vector(250,-250),dim=(500,-500)):
        '''
        Constructor MapContinous
        '''
        self.segments=[]
        self.origin=vOrigin
        self.dim=(500,-500)
    def append(self,segment):
        self.segments.append(segment)
    def getSegments(self):
        return self.segments
    def intersectionPoints(self,line):
        #return all intersection points from line in this map
        points=[]
        for s in self.segments:
            p=s.intersection(line)
            points.append(p)
        return points
    def nearestIntersectionPoint(self,line):
        pOrigin=line.getP()
        minPoint=vector(10e100,10e100)
        minD=10e100
        for s in self.segments:
            try:
                p=s.intersectLine(line)
                if line.isAfterOrigin(p):
                    d=(pOrigin-p).mag
                    if d<minD:
                        minD=d
                        minPoint=p
            except (PointNotIntersectSegments,NoIntersectLines):
                pass
        return minPoint
