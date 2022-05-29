#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np

def rotXYZ(rx,ry,rz, degrees =True):
    if(degrees):
        rx *= math.pi/180
        ry *= math.pi/180
        rz *= math.pi/180
    rotx = np.array([[1,0,0],
                     [0,math.cos(rx),-math.sin(rx)],
                     [0,math.sin(rx),math.cos(rx)]])
    
    roty = np.array([[math.cos(ry),0,math.sin(ry)],
                     [0,1,0],
                     [-math.sin(ry),0, math.cos(ry)]])
    
    rotz = np.array([[math.cos(rz),-math.sin(rz),0],
                     [math.sin(rz),math.cos(rz),0],
                     [-math.sin(ry),0,math.cos(ry)]])
    rotxyz = rotx.dot(roty.dot(rotz))
    return rotxyz

# define the transformations from one frame c to frameo
def homogenousMatrix(tx,ty,tz,rx,ry,rz):
    cRo = rotXYZ(rx,ry,rz, degrees =True)
    cTo = np.array([[tx,ty,tz]])
    M = np.concatenate(
        (np.concatenate((cRo,cTo.T), axis=1),
         np.array([[0,0,0,1]])),axis=0 )
    print(M)
    return M

# skew a 3x1 vector in a matrix
def skew_vec (vec):
    mat = np.array([[0,-vec[2], vec[1]],[vec[2],0,-vec[0]],[-vec[1], vec[0], 0]])
    return mat


# expressed a velocity in frame a knowing velocity in b an
# change of frame aMb
def velocityTwistMatrix(tx,ty,tz,rx,ry,rz):
    aRb = rotXYZ(rx,ry,rz, degrees=True)
    aTb_x = skew_vec(np.array([tx,ty,tz]))
    aTb_xaRb = aTb_x.dot(aRb)
    aVb1 = np.concatenate((aRb,aTb_xaRb),axis=1)
    aVb2 = np.concatenate((np.zeros((3,3)),aRb),axis=1) 
    aVb = np.concatenate ((aVb1,aVb2))
    return aVb
