#!/usr/bin/env python
import numpy as np
from transform import skew_vec
from transform import velocityTwistMatrix
from transform import homogenousMatrix

t = np.array([1,2,3])
r = np.array([90,10,20])
#mat = skew_vec(t)
#print type (mat)
#print  (mat)

#print 'homogenousMatrix', homogenousMatrix(t[0],t[1],t[2],r[0],r[1],r[2])

print 'twist', np.shape(velocityTwistMatrix(t[0],t[1],t[2],r[0],r[1],r[2]))

