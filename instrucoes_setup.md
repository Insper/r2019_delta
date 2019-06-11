
### Gazebo Turtlebot

Certifique-se de que seu `.bashrc` têm as variáveis `ROS_IP` e `ROS_MASTER_URI` desabilitadas antes de rodar o Gazebo


Você pode iniciar o Gazebo usando **uma** das opções:

    roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch

    roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch

    roslaunch turtlebot3_gazebo turtlebot3_stage_2.launch

    roslaunch turtlebot3_gazebo turtlebot3_stage_3.launch

    roslaunch turtlebot3_gazebo turtlebot3_house.launch

    roslaunch turtlebot3_gazebo turtlebot3_world.launch

O Gazebo já abre um `roscore` implicitamente




Executar `catkin_make` após fazer o download do projeto: 

    cd ~/catkin_ws/
    catkin_make


Certifique-se de que seus scripts Python são executáveis

    roscd p1_delta
    cd scripts
    chmod a+x *py

Para executar, faça:

    rosrun p1_delta p1_mobilenet.py 

ou 

    rosrun p1_delta p1_yolo.py 


Certifique-se de que seus scripts ROS rodam com Python 2 e têm sempre as linhas a seguir no começo:

    #! /usr/bin/env python
    # -*- coding:utf-8 -*-

### cv_camera

A maioria das webcams é o device 0, neste caso, se você tiver o cv_camera instalado:

    rosrun cv_camera cv_camera_node

Também precisamos fazer o relay:

    rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera

**Atenção:** feche o `rosrun cv_camera cv_camera_node` quando voltar a trabalhar na Questão 1 para não bloquear a webcam.


## Imutils

Você vai precisar do `imutils` para usar o Yolo. Para instalar:

    pip install imutils

Ou

    pip3 install imutils

