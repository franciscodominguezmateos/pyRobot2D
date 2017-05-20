#!/usr/bin/python
# -*- coding: utf-8 -*-
try:
    import cPickle as pickle
except ImportError:
    import pickle
import sys
from math import pi
from visual import vector
from Maps import MapContinous
from TinyGeometry.Line import Segment,Line
from PyQt4 import QtGui, QtCore
from Robots import Robot
from Sensors import SensorLIDAR
from Kinematics import KinematicTurtle,KinematicBicycle
from particles import Particles
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
        self.map.segments.append(Segment(vector(0,0),vector(320,0)))
        self.map.segments.append(Segment(vector(320,0),vector(320,240)))
        self.map.segments.append(Segment(vector(320,240),vector(0,240)))        
        self.map.segments.append(Segment(vector(0,240),vector(0,0)))  
         
        self.map.segments.append(Segment(vector(0,90),vector(50,90)))  
        self.map.segments.append(Segment(vector(50,90),vector(50,150)))  
        self.map.segments.append(Segment(vector(50,150),vector(0,150)))  
         
        self.map.segments.append(Segment(vector(270,0),vector(270,50)))  
        self.map.segments.append(Segment(vector(270,50),vector(320,50)))
         
        self.map.segments.append(Segment(vector(230,120),vector(320,120)))  
        self.map.segments.append(Segment(vector(230,120),vector(230,180)))  
        self.map.segments.append(Segment(vector(230,180),vector(320,180)))
        #f=file('map0.dat')
        #p=pickle.load(f)
        #f.close()
        #self.map.segments=getSegments(p)
        self.s=SensorLIDAR(8)
        self.k=KinematicTurtle()
        #self.k=KinematicBicycle()
        self.r=Robot(self.s,self.k,self.map)
        self.r.pos=vector(160,120)
        self.p=self.s.sensePoints(self.r)
        self.pt=Particles(self.r,500)
        self.initUI()
    def initUI(self):              
        self.setGeometry(300, 300, 320, 240)
        self.setWindowTitle('pyRobot2D')
        self.show()
    def changeTitle(self, state):
        self.r=self.r.move(0.0,10)
        self.p=self.s.sensePoints(self.r)
        self.pt.prediction(0.0,10)
        print self.pt.eval(self.r)
        if state == QtCore.Qt.Checked:
            self.setWindowTitle('Checkbox')
        else:
            self.setWindowTitle('')
        self.repaint()
    def changeTitle1(self, state):
        #self.r=self.r.move(0.2,10)
        #self.pt.run(self.r,0.2,10)
        Z = self.r.sense()
        self.pt.correction(Z)
        print self.pt.eval(self.r)
        #self.p=self.s.sensePoints(self.r)
        if state == QtCore.Qt.Checked:
            self.setWindowTitle('Checkbox')
        else:
            self.setWindowTitle('')
        self.repaint()
        
    def paintEvent(self, e):     
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setRenderHint(QtGui.QPainter.Antialiasing, True)
        qp.setPen(QtGui.QPen(QtGui.QColor(0,0,0,255)))
        for s in self.map.segments:
            xo=s.getOrigin().x
            yo=240-s.getOrigin().y
            xe=s.getEnd().x
            ye=240-s.getEnd().y
            qp.drawLine(xo,yo,xe,ye)
        xo=self.r.pos.x
        yo=240-self.r.pos.y
        qp.setPen(QtGui.QPen(QtGui.QColor(180,255,0,255)))
        for d,p in self.p:
            xe=p.x
            ye=240-p.y
            qp.drawLine(xo,yo,xe,ye)
        d,p=self.p[0]
        xe=p.x
        ye=240-p.y
        qp.setPen(QtGui.QPen(QtGui.QColor(128,128,0,255)))
        qp.drawLine(xo,yo,xe,ye)
        qp.setPen(QtGui.QPen(QtGui.QColor(0,255,0,255)))
        for r in self.pt.getParticles():
            x=r.pos.x
            y=240-r.pos.y
            qp.drawEllipse(x,y,5,5)           
        qp.end()
                   
def main():
    app = QtGui.QApplication(sys.argv)
    ex = wg()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()        