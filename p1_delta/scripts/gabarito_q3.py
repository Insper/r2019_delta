#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_empty_world.launch 


from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations
import random
import sys


x = None
y = None

contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

if __name__=="__main__":

    rospy.init_node("exemplo_odom")

    t0 = rospy.get_rostime()


    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))


   
    try:

        while not rospy.is_shutdown():
            # problemas de buffer?
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            # 

            angulo = 2*math.pi*ra      velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            # ndom.random()
            print("Angulo", angulo, " graus ", math.degrees(angulo))
            w = 2.5
            # Rotacao
            vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
            tempo = angulo/w
            print("tempo ", tempo)
            velocidade_saida.publish(vel)
            rospy.sleep(tempo)
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            # translacao
            vlin = 0.3
            linear = 1.33 # sympy
            tempo_lin = linear/vlin
            vel = Twist(Vector3(vlin,0,0), Vector3(0,0,0))
            rospy.sleep(tempo_lin)
            velocidade_saida.publish(zero)
            rospy.sleep(0.01)
            rospy.sleep(0.5)





            #rospy.sleep(0.5)
            #sys.exit()
    except rospy.ROSInterruptException:
            print("Ocorreu uma exceção com o rospy")



