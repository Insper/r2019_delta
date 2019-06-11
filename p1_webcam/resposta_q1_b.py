#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np

cap = cv2.VideoCapture('cat.m4v')
#cap = cv2.VideoCapture(0) # webcam

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    print("Codigo de retorno", ret)

    # Our operations on the frame come here
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    v1 = np.array([50,50,50])
    v2 = np.array([60,255,255])

    selecao = cv2.inRange(hsv, v1, v2)

    selecao_rgb = cv2.cvtColor(selecao, cv2.COLOR_GRAY2BGR)


    img_out, contornos, arvore = cv2.findContours(selecao.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 



    maior = None
    maior2 = None

    maior_area = 0

    for c in contornos:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area = area
            maior2 = maior
            maior = c

    contornos_img = selecao_rgb

    if maior is not None:
        cv2.drawContours(contornos_img, [maior], -1, [0, 255, 255], 5);
    if maior2 is not None:
        cv2.drawContours(contornos_img, [maior2], -1, [0, 255, 255], 5);

    # Display the resulting frame
    cv2.imshow('frame',contornos_img)
    #cv2.imshow('frame',dst)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
