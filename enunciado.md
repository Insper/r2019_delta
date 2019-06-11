# Robótica Computacional

## Avaliação Delta da P1

Observações de avaliações nesta disciplina:
* Esta prova tem 10 pontos, mas seu efeito é de substituir a P1 anterior com no máximo conceito $5.0$

Orientações gerais:
* Voce tem dois robôs disponíveis. Conte com apenas 20 minutos efetivos de funcionamento ou 30-40 minutos de *standby* para cada robô. Não deixe seu robô ligado sem necessidade
* Todas as questões podem ser feitas com robô real ou simulado
* Você pode consultar a *Internet* livremente, mas não pode se comunicar com outras pessoas da turma ou de fora dela sobre o conteúdo da prova. Tentativas de comunicação serão severamente punidas.
* Rode o script `apaga_para_entrega.sh` antes de enviar
* Ao final da prova, compacte a pasta com todo o seu código e envie pelo Blackboard.
* A responsabilidade por ter o *setup* funcionando é de cada estudante
* Haverá [uma planilha](https://docs.google.com/spreadsheets/d/1-f1smy-VNqcitWqFRtW1ErIKmvtqAsuE-rtw91epwu0/edit?usp=sharing)  compartilhada com fila para dúvidas. Indique nela se seu problema é de **infra** ou **geral**

Existe algumas dicas de referência rápida de setup [instrucoes_setup.md](instrucoes_setup.md)




# Questões


## Questão 1

Você foi contratado para desenvolver um sistema de visão computacional para um robô que vai trabalhar com colheita de toranjas.

O cliente passou a você uma escala de cores que identifica quais toranjas são verdes e quais são maduras.

A linha azul determina a partir de qual cor você pode considerar uma toranja madura

![](escala.png)

Considere que a componente `H` (Hue) das imagens vai definir o critério se uma fruta é madura ou não.

![](prova_laranjas.png)



Pede-se: Faća um código que conte e identifique na  imagem quantas toranjas maduras estão presentes.



Dicas:
* Lembre-se da aula 2
* Só precisa funcionar **para esta vídeo em particular**, não para quaisquer toranjas
* Não é uma questão de ROS. Trabalhe na pasta `p1_webcam`
* Você pode usar Python 2 ou Python 3 conforme preferir


|Resultado| Conceito| 
|---|---|
| Não executa | 0 |
| Dá falso positivo para outros círculos ou outras cores  ou não apresenta contornos | 1.5 |
| Apresenta o contorno que praticamente cobre os círculos verde | 2.75 |
| Apresenta um contorno desenhado sobre os círculos verdes da folha | 4.5 | 

Casos intermediários ou omissos da rubrica serão decididos pelo professor.


## Questão 2

Você vai receber uma caixa avermelhada.

Faca um programa que imprime na imagem a distância entre a caixa vermelha e sua webcam.

Você pode assumir que a caixa é mantida sempre na vertical

Assuma que a resolućão da webcam é `640 x 480`  e que a imagem abaixo foi obtida com a caixa a uma distância de 100cm da webcam.

![](caixa_calibracao.png)


|Resultado| Conceito| 
|---|---|
| Não executa | 0 |
| Consegue calcular o centro do gato e o centro da imagem | 1.5 |
| Realiza ações no sentido de centralizar, mas não converge ou a rotação não é no sentido ótimo | 2.75 |
| Gira até convergir para o gato | 4.5 | 

Casos intermediários ou omissos da rubrica serão decididos pelo professor.



## Questão 3 - ROS

Faća um programa em ROS que realize as seguintes tarefas:

* Sorteia um ângulo $\alpha$

* Gira o robô uma magnitude $\alpha$ no sentido horário

* Faz o robô comećar a andar em frente (em suas coordenadas locais)

* Usa a odometria (tópico `\odom`) para deixar o robô imóvel depois que este andou $1.33m$ em relaćão a sua posićào inicial

# Questão 4 - ROS + cv

**Atenćão: você vai usar OpenCV mas não vai trabalhar com imagens de câmera**

Você deve trabalhar no arquivo `le_scan_grafico.py`

O que você deve fazer:
* Leia os dados do *lidar* 
* Represente o robô na coordenada 256,256 da imagem usando um círculo
* Adotando a escala $1 pixel = 2 cm$ desenhe todas as leituras válidas do lidar na imagem



