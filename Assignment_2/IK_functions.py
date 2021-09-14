#! /usr/bin/env python3

"""
    # {Sebastian Granlund}
    # {sgran@kth.se}
"""
import math as math

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    #create q for later assigning
    q = [0.0, 0.0, 0.0]

    #Scaras links
    L0 = 0.07
    L1 = 0.3
    L2 =0.35
    #L0 is just a offset
    x = x - L0
    #Calculate the angle theta2 for second link
    cosTheta2 = (x**2 + y**2 -L1**2-L2**2)/(2*L1*L2)
    theta2 = math.acos(cosTheta2)
    #Calculate the angle theta1 for first link
    theta1 = math.atan2(y,x) - math.atan2(L2*math.sin(theta2), L1 + L2*cosTheta2)

    q[0] = theta1
    q[1] = theta2
    q[2] = z
    
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
