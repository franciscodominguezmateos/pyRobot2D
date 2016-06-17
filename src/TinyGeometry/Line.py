from visual import vector
import numpy as np
from  LineExceptions import *
"""
From Parametric Equation to General Equation
x=v.x*t+p.x -> t=(x-p.x)/v.x
y=v.y*t+p.y -> t=(y-p.y)/v.y
  (x-p.x)*v.y=(y-p.y)*v.x
x*v.y-p.x*v.y=y*v.x-p.y*v.x
x*v.y-y*v.x-p.x*v.y+p.y*v.x=0
v.y*x-v.x*y-p.x*v.y+p.y*v.x=0
 a *x+ b *y+      c        =0
a=v.y   b=-v.x  c=-p.x*v.y+p.y*v.x
"""
class Line:
    def __init__(self,p0,p1):
        self.p=vector(p0)
        self.v=vector(p1-p0).norm()
        self.a,self.b,self.c=self.workOutGeneralEquation()
        self.vg=vector(self.a,self.b,self.c)
    def __str__(self):
        return self.p.__str__() + self.v.__str__()
    def getV(self):
        return self.v
    def getP(self):
        return self.p
    def setV(self,v):
        self.v=v
        self.a,self.b,self.c=self.workOutGeneralEquation()
        self.vg=vector(self.a,self.b,self.c)
    def setP(self,p):
        self.p=p
        self.a,self.b,self.c=self.workOutGeneralEquation()
        self.vg=vector(self.a,self.b,self.c)
    def canIntersect(self,l1):
        v=self.getV().cross(l1.getV())
        if(v.mag==0):
            return False
        else:
            return True
    def workOutGeneralEquation(self):
        v=self.v
        p=self.p
        a= v.y
        b=-v.x
        c=-p.x*v.y+p.y*v.x
        return (a,b,c)
    def getGeneralEquation(self):
        return (self.a,self.b,self.c)
    def getParametricEquation(self):
        return (self.p,self.v)
    def distance(self,point):
        point.z=1
        return abs(self.vg.dot(point))
    def distanceSigned(self,point):
        point.z=1
        return self.vg.dot(point)
    def isForwardPoint(self,point):
        dp=point-self.p
        d=self.v.dot(dp)
        return d>=0
    def isBackwardPoint(self,point):
        dp=point-self.p
        d=self.v.dot(dp)
        return d<0
    def contains(self,point,tolerance=0.001):
        return self.distance(point)<tolerance
    def intersect(self,lin1):
        if(self.canIntersect(lin1)):
            (a0,b0,c0)= self.getGeneralEquation()
            (a1,b1,c1)= lin1.getGeneralEquation()
            a=np.array([[a0,b0],[a1,b1]])
            b=np.array([-c0,-c1])
            x=np.linalg.solve(a,b)
            return vector(x)
        else:
            raise NoIntersectLines("No Intersect Lines")
    def isAfterOrigin(self,p):
        ''' given p is a line point'''
        vp=p-self.p
        d=vp.dot(self.v)
        return d>0
class Segment:
    def __init__(self,p0,p1):
        self.v0=vector(p0)
        self.v1=vector(p1)
        self.line=Line(self.v0,self.v1)
    def __str__(self):
        return self.v0.__str__() + self.v1.__str__()
    def getOrigin(self):
        return self.v0
    def getEnd(self):
        return self.v1
    def getLength(self):
        return (self.v1-self.v0).mag
    def getLine(self):
        return self.line
    def contains(self,point):
        l=self.getLine()
        if(l.contains(point)):
            #check just the case point==v0
            if(point.x<>self.v0.x or point.y<>self.v0.y):
                v=point-self.v0
                vn=v.norm()
                sn=self.getLine().getV().norm()
                #print "norma",vn,sn
                #print "mag",v.mag,self.getLength()
                #v must have the same direction as self.line.getV()
                #print "ddkd",abs(vn.x-sn.x)<0.0001
                if(abs(vn.x-sn.x)<0.0001 and abs(vn.y-sn.y)<0.0001):
                    b=v.mag<=self.getLength()
                    #print "bool2",b
                    return b
                else:
                    return False
            else:
                return True
    def canIntersect(self,s1):
        l0=self.getLine()
        l1=s1.getLine()
        if(l0.canIntersect(l1)):
            v=l0.intersect(l1)
            if(self.contains(v) and s1.contains(v)):
                return True
            else:
                return False
        else:
            return False
    def intersect(self,s1):
        l0=self.getLine()
        l1=s1.getLine()
        v=l0.intersect(l1)
        if(self.contains(v) and s1.contains(v)):
            return v
        else:
            raise PointNotIntersectSegments("Point Not in Intersected Segment "+v.__str__())
    def intersectLine(self,l1):
        if l1.isBackwardPoint(self.v0) and l1.isBackwardPoint(self.v1):
            raise PointNotIntersectSegments("Points of segment are in the back of the line")
        d0=l1.distanceSigned(self.v0)
        d1=l1.distanceSigned(self.v1)
        if d0*d1>0: #if both of the points in the segment are in the same side of the line
            raise PointNotIntersectSegments("Points in segment are in the same face of the line")            
        l0=self.getLine()
        v=l0.intersect(l1)
        if self.contains(v):
            return v
        else:
            raise PointNotIntersectSegments("Point Not in Intersected Segment "+v.__str__())
    def intersectCut(self,sNext):
        v=self.intersect(sNext)
        return (Segment(self.getOrigin(),v),Segment(v,sNext.getEnd()))


