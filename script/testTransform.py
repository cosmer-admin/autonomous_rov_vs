#!/usr/bin/env python
import numpy as np
from transform import *

t = np.array([1,2,3])
r = np.array([0,0,90])

rRc = rotXYZ(r[0],r[1],r[2])
print(rRc)
print(isRot(rRc,10))

print ('rxc', rRc.dot([1,0,0])) 
print ('ryc', rRc.dot([0,1,0])) 
print ('rzc', rRc.dot([0,0,1])) 

#mat = skew_vec(t)
#print type (mat)
#print  (mat)

#print 'homogenousMatrix', homogenousMatrix(t[0],t[1],t[2],r[0],r[1],r[2])

#print 'twist', np.shape(velocityTwistMatrix(t[0],t[1],t[2],r[0],r[1],r[2]))

