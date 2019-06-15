#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


ranges = None
minv = 0
maxv = 10

def scaneou(dado):
    global ranges
    global minv
    global maxv
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
    #print("Intensities")
    #print(np.array(dado.intensities).round(decimals=2))


def desenha(cv_image):
    """
        Use esta funćão como exemplo de como desenhar na tela
    """
    cv2.circle(cv_image,(256,256),64,(0,255,0),2)
    cv2.line(cv_image,(256,256),(400,400),(255,0,0),5)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(cv_image,'Boa sorte!',(0,50), font, 2,(255,255,255),2,cv2.LINE_AA)

def draw_lidar(cv_image, leituras):
    pass

def draw_hough(cv_image):
    pass


if __name__=="__main__":

    rospy.init_node("le_scan")

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    cv2.namedWindow("Saida")


    while not rospy.is_shutdown():
        print("Oeee")
        velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
        velocidade_saida.publish(velocidade)
        # Cria uma imagem 512 x 512
        branco_rgb = np.zeros(shape=[512, 512, 3], dtype=np.uint8)
        # Chama funćões de desenho
        draw_lidar(branco, ranges)
        draw_hough(cv_image)

        # Imprime a imagem de saida
        cv2.imshow("Saida", branco)
        cv2.waitKey(0)
        rospy.sleep(0.1)



