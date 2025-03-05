import movement as m
import learning as ln
import math

def deter(x = None, y = None, z = None, dur = None):    
    xv = None
    yv = None
    if dur != None:
        xv = x / dur
        yv = y / dur
        x = None
        y = None
    m.madagascar(x = x, y = y, z = z, xv = xv, yv = yv) 


