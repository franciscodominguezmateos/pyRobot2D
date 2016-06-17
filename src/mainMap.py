#!/usr/bin/python
# -*- coding: utf-8 -*-
try:
    import cPickle as pickle
except ImportError:
    import pickle
import sys
import time
from math import pi
from visual import vector
from Maps import MapContinous
from TinyGeometry.Line import Segment,Line
from PySide import QtGui, QtCore
from Robots import Robot
from Sensors import SensorLIDAR
from Kinematics import KinematicTurtle,KinematicBicycle
from particles import Particles
import numpy as np
'''
Created on 27/07/2014

@author: Francisco Dominguez
'''
def getSegments(p):
    seg=[]
    for i,pt in enumerate(p):
        if i<len(p)-1:
            x0=pt[0]
            y0=pt[2]
            x1=p[i+1][0]
            y1=p[i+1][2]
            v0=vector(x0,y0)
            v1=vector(x1,y1)
            s=Segment(v0,v1)
            seg.append(s)
    return seg
        
class wg(QtGui.QWidget):
    
    def __init__(self):
        super(wg, self).__init__()
        cb = QtGui.QCheckBox('Show title', self)
        cb.move(20, 20)
        cb.toggle()
        cb.stateChanged.connect(self.changeTitle)
        cb1 = QtGui.QCheckBox('Show title', self)
        cb1.move(40, 20)
        cb1.toggle()
        cb1.stateChanged.connect(self.changeTitle1)
 
        self.map=MapContinous()
        f=file('map0.dat')
        p=pickle.load(f)
        f.close()
        self.map.segments=getSegments(p)
        self.maxX=np.max(p[:,0])
        self.maxY=np.max(p[:,2])
        self.minX=np.min(p[:,0])
        self.minY=np.min(p[:,2])
        self.map.segments.append(Segment(vector(self.maxX,self.maxY),vector(self.maxX,self.minY)))
        self.map.segments.append(Segment(vector(self.maxX,self.minY),vector(self.minX,self.minY)))
        self.map.segments.append(Segment(vector(self.minX,self.minY),vector(self.minX,self.maxY)))
        self.map.segments.append(Segment(vector(self.minX,self.maxY),vector(self.maxX,self.maxY)))
        self.s=SensorLIDAR(8)
        self.k=KinematicTurtle()
        #self.k=KinematicBicycle()
        self.r=Robot(self.s,self.k,self.map)
        self.r.pos=vector(0,0)
        self.p=self.s.sensePoints(self.r)
        self.pt=Particles(self.r,100)
        self.initUI()
    def initUI(self):              
        self.setGeometry(300, 300, 640, 480)
        self.setWindowTitle('pyRobot2D')
        self.show()
    def changeTitle(self, state):
        t=time.time()
        self.r=self.r.move(0.0,10)
        self.p=self.s.sensePoints(self.r)
        self.pt.prediction(0.0,10)
        print "p:%f,%f" % (time.time()-t,self.pt.eval(self.r))
        if state == QtCore.Qt.Checked:
            self.setWindowTitle('Checkbox')
        else:
            self.setWindowTitle('')
        self.repaint()
    def changeTitle1(self, state):
        #self.r=self.r.move(0.2,10)
        #self.pt.run(self.r,0.2,10)
        t=time.time()
        Z = self.r.sense()
        self.pt.correction(Z)
        print "c:%f,%f" % (time.time()-t,self.pt.eval(self.r))
        #self.p=self.s.sensePoints(self.r)
        if state == QtCore.Qt.Checked:
            self.setWindowTitle('Checkbox')
        else:
            self.setWindowTitle('')
        self.repaint()
    def global2screen(self,x,y):
        wX=self.maxX-self.minX
        wY=self.maxY-self.minY
        wsX=self.width()
        wsY=self.height()
        pxX=wsX/wX
        pxY=wsY/wY
        oX=self.minX*pxX
        oY=self.minY*pxY
        sx=pxX*x-oX
        sy=pxY*y-oY
        return (sx,sy)
    def paintEvent(self, e):     
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)
        qp.setPen(QtGui.QPen(QtGui.QColor(0,0,0,255)))
        #paint map
        for s in self.map.segments:
            xo,yo=self.global2screen(s.getOrigin().x,s.getOrigin().y)
            xe,ye=self.global2screen(s.getEnd().x,s.getEnd().y)
            qp.drawLine(xo,yo,xe,ye)
        #paint lidar rays
        xo,yo=self.global2screen(self.r.pos.x,self.r.pos.y)
        qp.setPen(QtGui.QPen(QtGui.QColor(180,255,0,255)))
        for d,p in self.p:
            xe,ye=self.global2screen(p.x,p.y)
            qp.drawLine(xo,yo,xe,ye)
        d,p=self.p[0]
        xe,ye=self.global2screen(p.x,p.y)
        qp.setPen(QtGui.QPen(QtGui.QColor(128,128,0,255)))
        qp.drawLine(xo,yo,xe,ye)
        #paint particles
        qp.setPen(QtGui.QPen(QtGui.QColor(0,255,0,255)))
        for r in self.pt.getParticles():
            x,y=self.global2screen(r.pos.x,r.pos.y)
            qp.drawEllipse(x,y,5,5)           
        qp.end()
                   
def main():
    app = QtGui.QApplication(sys.argv)
    ex = wg()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()        