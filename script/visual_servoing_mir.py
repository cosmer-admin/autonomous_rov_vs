#!/usr/bin/env python
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken
import rospy
import tf
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist


###---- Visual Tracking and Servoing----
from Point2D import Point2D


import time
import sys
import argparse

# ---------- Global Variables ---------------
global enable_depth
global enable_vs
global init_p0
global depth_p0
global depth_wrt_startup
global desired_points
global vcam_vs
global lambda_vs
global n_points_vs

# visual servoing
lambda_vs = 0.5
n_points_vs = 8
desired_points = [0]

#camera parameters
u0 = 341
v0 = 258
lx = 455
ly = 455
kud =0.00683 
kdu = -0.01424 




    


global desired_points2D
#defined desired points 
#desired_points2D = [Point2D(-0.243369710525,-0.418902983183),
                  #Point2D(-0.0143862444929,-0.429675544624),
                  #Point2D(-0.096243540171,-0.352463371199),
                  #Point2D(-0.237090345635,-0.291995145385),
                  #Point2D(-0.156886982784,-0.189359449174),
                  #Point2D(-0.00989097068426,-0.148452126201),
                  #Point2D(-0.00165708972955,-0.0341907266445),
                  #Point2D(-0.224742039631,-0.0227008870907)]

desired_points2D = [Point2D(-0.243369710525,-0.418902983183),
                  Point2D(-0.0143862444929,-0.429675544624),
                  Point2D(-0.096243540171,-0.352463371199),
                  Point2D(-0.237090345635,-0.291995145385),
                  Point2D(-0.156886982784,-0.189359449174),
                  Point2D(-0.00989097068426,-0.148452126201),
                  Point2D(-0.00165708972955,-0.0341907266445),
                  Point2D(-0.224742039631,-0.0227008870907)]
    
vcam_vs = np.array([0,0,0,0,0,0])
lambda_vs = 0.5


set_mode = [0]*3
Vmax_mot = 1900
Vmin_mot = 1100

angle_wrt_startup = [0]*3
depth_wrt_startup = 0
depth_p0 = 0
rho = 1000.0
gravity = 9.80665

#Conditions
init_a0 = True
init_p0 = True
arming = False
custom_PI = False

set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

enable_depth = False 
enable_vs = 0   # set to tracker status
enable_ping = True 
pinger_confidence = 0
pinger_distance = 0

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 




def joyCallback(data):
    global arming
    global set_mode
    global set_mode
    global custom_PI

    # Joystick buttons
    btn_arm = data.buttons[7]  # Start button
    btn_disarm = data.buttons[6]  # Back button
    btn_manual_mode = data.buttons[3]  # Y button
    btn_automatic_mode = data.buttons[2]   # X button
    btn_corrected_mode = data.buttons[0]    # A button

    # Disarming when Back button is pressed
    if (btn_disarm == 1 and arming == True):
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if (btn_arm == 1 and arming == False):
        arming = True
        armDisarm(arming)

    # Switch manual, auto and correction mode
    if (btn_manual_mode and not set_mode[0]):
        set_mode[0] = True
        set_mode[1] = False
        set_mode[2] = False
        rospy.loginfo("Mode manual")
    if (btn_automatic_mode and not set_mode[1]):
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False
        rospy.loginfo("Mode automatic")
    if (btn_corrected_mode and not set_mode[2]):
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        custom_PI = True        
        rospy.loginfo("Mode correction")


def armDisarm(armed):
    # This functions sends a long command service with 400 code to arm or disarm motors
    if (armed):
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Arming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except arming")
    else:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Disarming Succeeded")
        except (rospy.ServiceException, e):
            rospy.loginfo("Except disarming")


def velCallback(cmd_vel):
    global set_mode

    # Only continue if manual_mode is enabled
    if (set_mode[1] or set_mode[2]):
        return

    # Extract cmd_vel message
    roll_left_right = mapValueScalSat(cmd_vel.angular.x)
    yaw_left_right = mapValueScalSat(-cmd_vel.angular.z)
    ascend_descend = mapValueScalSat(cmd_vel.linear.z)
    forward_reverse = mapValueScalSat(cmd_vel.linear.x)
    lateral_left_right = mapValueScalSat(-cmd_vel.linear.y)
    pitch_left_right = mapValueScalSat(cmd_vel.angular.y)

    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend,
                    yaw_left_right, forward_reverse, lateral_left_right)


def OdoCallback(data):
    global angle_roll_a0
    global angle_pitch_a0
    global angle_yaw_a0
    global angle_wrt_startup
    global init_a0
    global p
    global q
    global r
   
    orientation = data.orientation
    angular_velocity = data.angular_velocity

    # extraction of yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    angle_roll = euler[0]
    angle_pitch = euler[1]
    angle_yaw = euler[2]

    if (init_a0):
        # at 1st execution, init
        angle_roll_a0 = angle_roll
        angle_pitch_a0 = angle_pitch
        angle_yaw_a0 = angle_yaw
        init_a0 = False

    angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi) %
                            (2.0*math.pi) - math.pi) * 180/math.pi
    angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi) %
                            (2.0*math.pi) - math.pi) * 180/math.pi
    angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi) %
                            (2.0*math.pi) - math.pi) * 180/math.pi

    angle = Twist()
    angle.angular.x = angle_wrt_startup[0]
    angle.angular.y = angle_wrt_startup[1]
    angle.angular.z = angle_wrt_startup[2]

    pub_angle_degre.publish(angle)

    # Extraction of angular velocity
    p = angular_velocity.x
    q = angular_velocity.y
    r = angular_velocity.z

    vel = Twist()
    vel.angular.x = p
    vel.angular.y = q
    vel.angular.z = r
    pub_angular_velocity.publish(vel)



def mapValueScalSat(value):
    global Vmax_mot
    global Vmin_mot
    # Correction_Vel and joy between -1 et 1
    # scaling for publishing with setOverrideRCIN values between 1100 and 1900
    # neutral point is 1500
    pulse_width = value * 400 + 1500

    # On limite la commande en vitesse
    if pulse_width > Vmax_mot:
        pulse_width = Vmax_mot
    if pulse_width < Vmin_mot:
        pulse_width = Vmin_mot

    return pulse_width


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
    # This function replaces setservo for motor commands.
    # It overrides Rc channels inputs and simulates motor controls.
    # In this case, each channel manages a group of motors not individually as servo set

    msg_override = OverrideRCIn()

    msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
    msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll 	
    msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
    msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw		    
  
    msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge		
    
    msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500
    msg_override.channels[7] = 1500
    
    # print("<3=====D ",msg_override)
    pub_msg_override.publish(msg_override)



# Function used to calculate the necessary PWM for each motor
def PWM_Cmd(thrust_req):
    if (thrust_req >= 0):
        m = 86.93393326839376    # Slope of the positive force
        b = 1536
    else:
        m = 110.918185437553874  # Slope of the negtaive force 
        b = 1464

    PWM = int(m * thrust_req/4) + b
    if PWM > Vmax_mot:
        PWM = Vmax_mot
    if PWM < Vmin_mot:
        PWM = Vmin_mot

    return PWM



    

    

def computeError(desired_points, current_points):
    error = []
    if(np.shape(desired_points)==np.shape(current_points)):
        i = 0
        for point in current_points:
            error.append(desired_points[i].x- point.x)
            error.append(desired_points[i].y- point.y) 
            i = i + 1
    else :
        print("error current point and desired point vector not of the same dim")
        error = 0
    return error
    
    
    
        
def interactionMatrixFeaturePoint2D(x,y,Z=1):
    Lx = np.array([ -1 / Z, 0, x / Z, x * y,-(1 + x * x), y])
    Ly = np.array([0, -1 / Z, y / Z, 1 + y * y, -x * y, -x ])
    L = np.stack((Lx.T,Ly.T),axis=0)
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
    n = int(np.shape(points)[0]/2)
    point_reshaped = (np.array(points).reshape(n,2))
    point_meter = []
    for pt in point_reshaped:
        pt_meter = convert2meter(pt,u0,v0,lx,ly)
        point_meter.append(pt_meter)
    point_meter = np.array(point_meter).reshape(-1)
    return point_meter

def trackercallback(data):
    print "tracker ON"
    global desired_points
    global n_points_vs
    
    current_points = data.data
    current_points_meter = convertListPoint2meter (current_points)
    desired_points_meter = convertListPoint2meter (desired_points)
    
    error = np.array(current_points_meter)-np.array(desired_points_meter)
    L = interactionMatrixFeaturePoint2DList(current_points_meter, np.array([1]))
    
    vcam = -lambda_vs * np.linalg.pinv(L).dot(error)
    print(vcam)
    
    vel = Twist()
    vel.angular.x = vcam[3]
    vel.angular.y = vcam[4]
    vel.angular.z = vcam[5]
    pub_angular_velocity.publish(vel)
    
    Vel = Twist()
    Vel.linear.x = vcam[0]
    Vel.linear.y = vcam[1]
    Vel.linear.z = vcam[2]
    pub_linear_velocity.publish(Vel)
    


    
def desiredpointscallback(data):
    #print "desired point callback"
    global desired_points
    desired_points = data.data
    
    
    #print (data.data)
    #enable_vs = 1
    
    #if(np.size(points.data)==n_points_vs):
        #enable_vs = 1
        #print("enable vs !!")
    #else : 
        #enable_vs = 0
        #print("disable vs")
    
  
    
    #if(enable_vs ==1):
        #print ("tracker ok")
        
        
        #L_vs = interactionMatrixFeaturePoint2DList(current_points2D, np.array([1]))
        #error_vs = computeError(desired_points2D, current_points2D)
        
        #vcam_vs = -lambda_vs* np.linalg.pinv(L_vs).dot(error_vs)
        #print "vcam= ", vcam_vs.T
        
    

def subscriber():
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    
    #camera
    rospy.Subscriber("tracked_points",Float64MultiArray,trackercallback, queue_size=1)
    rospy.Subscriber("desired_points",Float64MultiArray,desiredpointscallback, queue_size=1)
    rospy.spin()  # Execute subscriber in loop


if __name__ == '__main__':

    #armDisarm(False)  # Not automatically disarmed at startup
    rospy.init_node('visual_servoing_mir', anonymous=False)
    print "visual servoing mir launched"
    pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size= 10, tcp_nodelay = True)
    pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)

    
    subscriber()


