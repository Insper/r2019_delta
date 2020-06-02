#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

import math


leituras = None

def scaneou(dado):
    global leituras
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    leitura_atual = np.array(dado.ranges).round(decimals=2)
    print(leitura_atual)
    leituras = leitura_atual.copy()
    # poderia guardar range_min e range_max
    #print("Intensities")
    #print(np.array(dado.intensities).round(decimals=2))


def desenha(cv_image):
    """
        Use esta funćão como exemplo de como desenhar na tela
    """
    #cv2.circle(cv_image,(256,256),64,(0,255,0),2)
    #cv2.line(cv_image,(256,256),(400,400),(255,0,0),5)
    #font = cv2.FONT_HERSHEY_SIMPLEX
    # cv2.putText(cv_image,'Boa sorte!',(0,50), font, 2,(255,255,255),2,cv2.LINE_AA)

    if leituras is not None:
        for i in range(len(leituras)): 
            dist = leituras[i]
            if 0.12 < dist < 3.5:
                ang = math.radians(i) # lembrar!
                cx = 256
                cy = 250
                y = cy - dist*math.sin(ang)*50
                x = cx - dist*math.cos(ang)*50
                cv2.circle(cv_image,(int(x), int(y)),3,(0,0,255),2)




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
        branco = np.zeros(shape=[512, 512, 3], dtype=np.uint8)
        # Chama funćões de desenho
        desenha(branco)
        # Imprime a imagem de saida
        cv2.imshow("Saida", branco)
        cv2.waitKey(1) # Precisa do waitkey 1 para ele *nao* esperar a key
        rospy.sleep(0.1)



