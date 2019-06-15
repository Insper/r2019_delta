#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import math


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
    if leituras is None:
        return
    bot = [256,256] # centro do robô
    escala = 50 # transforma 0.01m em 0.5 px

    raio_bot = int(0.1*escala)
    # Desenha o robot
    cv2.circle(cv_image,(bot[0],bot[1]),raio_bot,(255,0,0),1)

    for i in range(len(leituras)):
        rad = math.radians(i)
        dist = leituras[i]
        if minv < dist < maxv:
            xl = int(bot[0] + dist*math.cos(rad)*50)
            yl = int(bot[1] + dist*math.sin(rad)*50)
            cv2.circle(cv_image,(xl,yl),1,(0,255,0),2)


def draw_hough(cv_image):
    img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", img_gray)
    cv2.waitKey(10)
    # O trecho abaixo é copiado direto da aula 2, parte sobre Hough, colocando só ma checagem para None na primeira iteraćão
    lines = cv2.HoughLinesP(img_gray, 10, math.pi/180.0, 100, np.array([]), 45, 5)
    if lines is None:
        print("No lines found")
        return
    a,b,c = lines.shape
    for i in range(a):
        # Faz uma linha ligando o ponto inicial ao ponto final, com a cor vermelha (BGR)
        cv2.line(cv_image, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 1, cv2.LINE_AA)    



if __name__=="__main__":

    rospy.init_node("le_scan")

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    cv2.namedWindow("Saida")


    while not rospy.is_shutdown():
        # Cria uma imagem 512 x 512

        branco_rgb = np.zeros(shape=[512, 512, 3], dtype=np.uint8)
        # Chama funćões de desenho
        draw_lidar(branco_rgb, ranges)
        draw_hough(branco_rgb)

        # Imprime a imagem de saida
        cv2.imshow("Saida", branco_rgb)
        cv2.waitKey(40) # TRocamos o 0 por 40 para esperar 40 millisegundos
        rospy.sleep(0.1)



