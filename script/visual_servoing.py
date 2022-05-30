#!/usr/bin/env python
import numpy as np
    
        
def interactionMatrixFeaturePoint2D(x,y,Z=1):
    #TODO Mir Student : write the interaction matrix here
    L = np.zeros(2,6)
    return L


#list of points and list of Z
def interactionMatrixFeaturePoint2DList(points, Zs):
    n = int(np.shape(points)[0]/2)
    if(len(Zs)!=n):
       Zs = np.ones(n)
        
    iter = 0 
    L = [[]];
    # for all the points
    point_reshaped = (np.array(points).reshape(n,2))
    for p in point_reshaped:
        Lp = interactionMatrixFeaturePoint2D(p[0],p[1],Zs[iter])
        if(iter == 0) : 
            L = Lp
        else: 
            L = np.concatenate((L,Lp))
        iter += 1
    return L

def convert2meter(pt,u0,v0,lx,ly):
    return (pt[0]-u0)/lx, (pt[1]-v0)/ly

def convertListPoint2meter (points):
    global u0,v0,lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n,2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt,u0,v0,lx,ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter
