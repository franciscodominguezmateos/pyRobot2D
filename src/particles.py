from math import sqrt,pi
import random

# 
# 
# this is the particle filter class for robot location
#
class Particles:
    #
    # init: 
    #	creates particle set with given initial position
    def __init__(self,robot, N = 1000):
        self.N = N      
        self.particle = []
        for i in range(self.N):
            r = robot.copyRandom()
            self.particle.append(r)
    def getParticles(self):
        return self.particle
    def getParticle(self,i):
        return self.particle[i]
    # --------
    #
    # extract position from a particle set
    # 
    def get_position(self):
        x = 0.0
        y = 0.0
        orientation = 0.0

        for i in range(self.N):
            x += self.particle[i].x
            y += self.particle[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            orientation += (((self.particle[i].orientation
                              - self.particle[0].orientation + pi) % (2.0 * pi)) 
                            + self.particle[0].orientation - pi)
        return [x / self.N, y / self.N, orientation / self.N]

    # --------
    #
    # motion of the particles
    # 
    def move(self, steer, speed):
        newParticle = []
        for i in range(self.N):
            r = self.particle[i].move(steer, speed)
            newParticle.append(r)
        self.particle = newParticle    
        
    # --------
    #
    # sensing 
    # 
    def getWeight(self,Z):
        # Particle weight
        w = []
        for i in range(self.N):
            w.append(self.particle[i].measurement_prob(Z))
#            if i % 10 == 0:
#                print "Pw",i
        return w
    def sense(self, Z):
        # Particle weight
        w = self.getWeight(Z)
        return w
    # --------
    #
    # resampling
    # 
    def resampling(self,w):
        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)
        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.particle[index])
        self.particle = p3
        
    # --------
    #
    # eval actual robot pose by MSE
    # 
    def eval(self,robot):
        p=self.particle
        r=robot
        s = 0.0;
        for i in range(len(p)): # calculate mean error
            s += (p[i].pos-r.pos).mag #error
        return s / float(len(p))

    #run a particles filter iteration    
    def run(self,robot,steer,speed):
        robot=robot.move(steer, speed)
        self.prediction(steer, speed)
        Z = robot.sense()
        self.correction(Z)
    def prediction(self,steer,speed):
        self.move(steer,speed)
    def correction(self,Z):
        w=self.sense(Z)
        self.resampling(w)

