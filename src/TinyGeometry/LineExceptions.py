'''
Created on 23/03/2013

@author: Francisco Dominguez
'''
class NoIntersectLines(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
       return repr(self.value)
   
class PointNotIntersectSegments(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
       return repr(self.value)