#!/usr/bin/env python3

# Importing necessary Libraries
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist

##############
import sympy as sym
sym.init_printing()
from sympy import *

import numpy as np
from numpy import *
import math
################

#Initializing Variables

theta1, theta2, theta3, theta4, theta5, theta6, theta7 = sym.symbols("\\theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7")
d1, d3, d5, d7 = sym.symbols("d_1,d_3,d_5,d_7")

d1 = 360
d3 = 420
d5 = 399.5
d7 = 205.5

################## CHANGE MASS in Accordance ################
m1 = 4
m2 = 4
m3 = 4
m4 = 4
m5 = 4
m6 = 4

# Taking Gravitational Constant NEGATIVE to obtain positive Potential Energy
g = -9.81
################

def plotcircle():
    #Initializing Node
    rospy.init_node('publish_node', anonymous=True) # defining the ros node - publish node
    turning = rospy.Publisher('turn', Float64MultiArray, queue_size=10) 
    rate = rospy.Rate(10) # 10hz # fequency at which the publishing occurs
    rospy.loginfo("Analysing the Robot!!!")  # to print on the terminal

    # Obtaining Transformation Matrices
    A1 = sym.Matrix([[sym.cos(theta1), 0, -sym.sin(theta1), 0], [sym.sin(theta1), 0, sym.cos(theta1), 0], [0, -1, 0, d1], [0, 0, 0, 1]])
    A2 = sym.Matrix([[sym.cos(theta2), 0, sym.sin(theta2), 0], [sym.sin(theta2), 0, -sym.cos(theta2), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    A3 = sym.Matrix([[sym.cos(0), 0, sym.sin(0), 0], [sym.sin(0), 0, -sym.cos(0), 0], [0, 1, 0, d3], [0, 0, 0, 1]])
    A4 = sym.Matrix([[sym.cos(theta4), 0, -sym.sin(theta4), 0], [sym.sin(theta4), 0, sym.cos(theta4), 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    A5 = sym.Matrix([[sym.cos(theta5), 0, -sym.sin(theta5), 0], [sym.sin(theta5), 0, sym.cos(theta5), 0], [0, -1, 0, d5], [0, 0, 0, 1]])
    A6 = sym.Matrix([[sym.cos(theta6), 0, sym.sin(theta6), 0], [sym.sin(theta6), 0, -sym.cos(theta6), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    A7 = sym.Matrix([[sym.cos(theta7), -sym.sin(theta7), 0, 0], [sym.sin(theta7), sym.cos(theta7), 0, 0], [0, 0, 1, d7], [0, 0, 0, 1]])

    A = A1*A2*A3*A4*A5*A6*A7
    A12 = A1*A2
    A24 = A12*A3*A4
    A45 = A24*A5
    A56 = A45*A6
    A67 = A56*A7

    # Obtaining Z Matrices from Transformation Matrix
    Z0 = sym.Matrix([0,0,1])
    Z1 = A1[:3,2]
    Z2 = A12[:3,2]
    Z4 = A24[:3,2]
    Z5 = A45[:3,2]
    Z6 = A56[:3,2]
    Z7 = A67[:3,2]
	
    #Obtaining O Matrix (Translation) from Transformation Matrix
    O0 = sym.Matrix([0, 0, 0])
    O1 = A1[:3,3]
    O2 = A12[:3,3]
    O4 = A24[:3,3]
    O5 = A45[:3,3]
    O6 = A56[:3,3]
    O7 = A67[:3,3]

    # Calculating Jacobian
    px = A[0,3]; py = A[1,3]; pz = A[2,3];
    a11 = sym.diff(px, theta1)
    a12 = sym.diff(px, theta2)
    a13 = sym.diff(px, theta4)
    a14 = sym.diff(px, theta5)
    a15 = sym.diff(px, theta6)
    a16 = sym.diff(px, theta7)

    a21 = sym.diff(py, theta1)
    a22 = sym.diff(py, theta2)
    a23 = sym.diff(py, theta4)
    a24 = sym.diff(py, theta5)
    a25 = sym.diff(py, theta6)
    a26 = sym.diff(py, theta7)

    a31 = sym.diff(pz, theta1)
    a32 = sym.diff(pz, theta2)
    a33 = sym.diff(pz, theta4)
    a34 = sym.diff(pz, theta5)
    a35 = sym.diff(pz, theta6)
    a36 = sym.diff(pz, theta7)

    J = sym.Matrix([[a11, a12, a13, a14, a15, a16], [a21, a22, a23, a24, a25, a26],[a31, a32, a33, a34, a35, a36],[Z1,Z2,Z4,Z5,Z6,Z7]])

    # Calculate Potential Energy
    P1 = -1*m1*g*(O1[2]+O0[2])*0.5
    P2 = -1*(m1+m2)*g*(O2[2]+O1[2])*0.5
    P3 = -1*(m1+m2+m3)*g*(O4[2]+O2[2])*0.5
    P4 = -1*(m1+m2+m3+m4)*g*(O5[2]+O4[2])*0.5
    P5 = -1*(m1+m2+m3+m4+m5)*g*(O6[2]+O5[2])*0.5
    P6 = -1*(m1+m2+m3+m4+m5+m6)*g*(O7[2]+O6[2])*0.5

    P = sym.Matrix([[P1], [P2], [P3], [P4], [P5], [P6]])
    
    # Wrench Vector for FORCE
    Fw = sym.Matrix([[0], [-5], [0], [0], [0], [0]])

    # # Inverse Kinematics
    # theta_joint = sym.Matrix([0,30,-45,0,75,0])*(pi/180)
    # N = 60
    # th = linspace(float(pi/2), float((5*pi)/2),num=N)

    while not rospy.is_shutdown():

        # Inverse Kinematics
        theta_joint = sym.Matrix([0,75,-60,60,45,0])*(pi/180)
        N = 60
        th = linspace(float(pi/2), float((5*pi)/2),num=N)

        T1 = []
        T2 = []
        T3 = []
        T4 = []
        T5 = []
        T6 = []
        cirx = []
        ciry = []

        old_min = -0.88
        old_max = 2.00
        new_min = -0.5
        new_max = 0.5
    
        for i in range(0,N):
            twist = Float64MultiArray()

            x_dot = -100.0 * (2*pi/5)* sin(th[i])  
            z_dot = 100.0 * (2*pi/5)* cos(th[i])

            V = Matrix([x_dot,0.0, z_dot, 0.0, 0.0, 0.0])

            J_inv = J.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta4:theta_joint[2],theta5:theta_joint[3],theta6:theta_joint[4],theta7:theta_joint[5]}).inv()

            theta_dot = J_inv*V

            theta_joint = theta_joint + (theta_dot*(5/N))

            #T = A.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta4:theta_joint[2],theta5:theta_joint[3],theta6:theta_joint[4],theta7:theta_joint[5]})
            #PP = P.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta4:theta_joint[2],theta5:theta_joint[3],theta6:theta_joint[4],theta7:theta_joint[5]})

            #Torque = PP - J.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta4:theta_joint[2],theta5:theta_joint[3],theta6:theta_joint[4],theta7:theta_joint[5]}).T * Fw

            #new_value = ( (theta_joint - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min

            
            print(theta_joint)
            twist.data = theta_joint
            #print(test)
            turning.publish(twist)
            rate.sleep()
        
    #plt.scatter(T[0,3],T[2,3])

if __name__ == '__main__':
    try:
        plotcircle()
    except rospy.ROSInterruptException: 
        pass
