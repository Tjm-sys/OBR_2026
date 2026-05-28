#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor # pyright: ignore[reportMissingImports]
from pybricks.parameters import Port, Stop, Direction, Color # pyright: ignore[reportMissingImports]
from pybricks.tools import wait # pyright: ignore[reportMissingImports]
from pybricks.robotics import DriveBase # pyright: ignore[reportMissingImports]

#variaveis dos componentes
motor_direita = Motor(Port.A) 
motor_esquerda = Motor(Port.B)
motor_garra = Motor(Port.C) 
motor_conteiner = Motor(Port.D)
sensor_direita = ColorSensor(Port.S1)
sensor_esquerda = ColorSensor(Port.S2)
sensor_giro = GyroSensor(Port.S3)
sensor_ultrasonico = UltrasonicSensor(Port.S4)

#Configuracoes iniciais
robot = DriveBase(motor_esquerda, motor_direita, wheel_diameter=55, axle_track=104) #ajustar conforme o robo
potencia = 80    #velocidade do robo em porcentagem nas linhas
temp_resgate = 120 #tempo programdo para fazer o resgate
vitima = 0 #vitima localizada 0/1

def mover_bloco_sgiro(potencia, num_graus_distancia): #bloco para movimento reto usando sensor
    sensor_giro.reset_angle(0)    #reseta valores

    while abs(motor_esquerda.angle()) < num_graus_distancia:   
        diferenca_bloco_mov = sensor_giro.angle()
        correcao_bloco_mov = diferenca_bloco_mov * 4.5   #Ajustar o valor "4.5" se necessario
        motor_esquerda.dc(potencia - correcao_bloco_mov)
        motor_direita.dc(potencia + correcao_bloco_mov)
    
    robot.stop()
    wait(100)

def virar_bloco_sgiro(graus_bloco_giro, direcao_bloco_giro, potencia_do_giro): #bloco para virar usando sensor
    robot.stop()
    wait(100)
    sensor_giro.reset_angle(0)    #reseta valor
    
    if direcao_bloco_giro == "direita":
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            robot.drive(potencia_do_giro, potencia_do_giro)
            #motor_esquerda.dc(potencia_do_giro)
            #motor_direita.dc(-potencia_do_giro)
            wait(10)

    elif direcao_bloco_giro == "esquerda":
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            robot.drive(potencia_do_giro, -potencia_do_giro)
            #motor_esquerda.dc(-potencia_do_giro)
            #motor_direita.dc(potencia_do_giro)
            wait(10)

    robot.stop()
    wait(100)

def virar_180_bloco_sgiro(graus_bloco_giro, potencia_do_giro): #bloco para movimento 180 usando sensor
    robot.stop()
    wait(100)
    sensor_giro.reset_angle(0)    #reseta valor

    while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
        #robot.drive(potencia_do_giro, -potencia_do_giro)
        motor_esquerda.dc(potencia_do_giro)
        motor_direita.dc(-potencia_do_giro)
        wait(10)

    robot.stop()
    wait(100)

def mostrar(): #funcao para testes, mostrando os valores enquanto o codigo esta rodando
    cen_sensor_ultrasonico = sensor_ultrasonico.distance()
    cor_sensor_direita = sensor_direita.color()
    cor_sensor_esquerda = sensor_esquerda.color()
    ref_sensor_direita = sensor_direita.reflection()
    ref_sensor_esquerda = sensor_esquerda.reflection()
    
    print("Cor do sensor direito: {}".format(cor_sensor_direita))
    print("Cor do sensor esquerdo: {}".format(cor_sensor_esquerda))
    print("Reflexo do sensor direito: {}".format(ref_sensor_direita))
    print("Reflexo sensor esquerdo: {}".format(ref_sensor_esquerda))
    print("Proximidade de objetos (sensor): {}".format(cen_sensor_ultrasonico))

def caixa(): #desvio para o obstaculo
        virar_bloco_sgiro(90, "direita", 100)
        mover_bloco_sgiro(80, 100)
        virar_bloco_sgiro(90, "esquerda", 100)
        mover_bloco_sgiro(80, 240)
        virar_bloco_sgiro(90, "esquerda", 100)
        mover_bloco_sgiro(80, 200)
        virar_bloco_sgiro(90, "direita", 100)

while True:
    if sensor_ultrasonico.distance() < 120:  #distancia em mm
        caixa()
    elif sensor_direita and sensor_esquerda == "silver":
        mover_bloco_sgiro(80, 120) #*45cm
        virar_bloco_sgiro(90, "esquerda", 80)
        mover_bloco_sgiro(80, 120) #*45cm

        while temp_resgate <= 120:
            if sensor_ultrasonico.distance() >= 30 and vitima == 0:
                robot.drive(40, -40, 40)
                dist_vitima = sensor_ultrasonico.distance()
            else:
                vitima == 1
                robot.drive.stop()
                robot.drive(40, dist_vitima - 5, (dist_vitima - 5))
                wait(50)
                motor_garra.on_for_degrees(speed=30, degrees=90) #pega vitima
                robot.drive(40, (dist_vitima - 5)*(-1), (dist_vitima - 5)*(-1))
                wait(50)
                vitima == 0
                
    else:
        #mostrar()
        if sensor_direita.color() == Color.RED: #and sensor_esquerda.color() != Color.GREEN:
            print("vermelho")
            motor_direita.stop()
            motor_esquerda.stop()

        elif sensor_direita.color() == Color.GREEN and sensor_esquerda.color() == Color.GREEN:
            print("verde nos dois")
            virar_180_bloco_sgiro(180, 80)

        elif sensor_direita.color() == Color.GREEN: #and sensor_esquerda.color() != Color.GREEN:
            wait(50)
            if sensor_direita.color() == Color.GREEN: #and sensor_esquerda.color() == Color.GREEN:
                print("verde na esquerda")
                virar_bloco_sgiro(85, "esquerda", 80)

        elif sensor_esquerda.color() == Color.GREEN: #and sensor_direita.color() != Color.GREEN:
            wait(50)
            if sensor_esquerda.color() == Color.GREEN: #and sensor_direita.color() == Color.GREEN:
                print("verde na direita")
                virar_bloco_sgiro(85, "direita", 80)

        else:
            wait(5)
            #calculo central
            ref_sensor_direita = sensor_direita.reflection()
            ref_sensor_esquerda = sensor_esquerda.reflection()

            erro_anterior = 0
            integral = 0
            kp = 4.2 #oscila muito -> diminuir KP; responde lento -> aumentar KP; tremendo -> aumentar KD; puxando para um lado -> ajustar OFFSET; erro continuo -> aumentar KI;
            ki = 0.02
            kd = 1.2

            ref_erro = ref_sensor_direita - ref_sensor_esquerda
            integral += ref_erro #integral
            derivada = ref_erro - erro_anterior #derivado

            correcao = (kp * ref_erro) + (ki * integral) + (kd * derivada) #PID
            erro_anterior = ref_erro #salva erro para o derivado

            velocidade = potencia - abs(correcao) * 0.6
            velocidade = min(velocidade ,50)
            if velocidade < 50: velocidade = 50

            robot.drive(velocidade, correcao)

            #ref_erro = ref_sensor_direita - ref_sensor_esquerda #somente P
            #correcao = kp * ref_erro

            #velocidade_linear = potencia
            #velocidade_angular = correcao
            #robot.drive(velocidade_linear, velocidade_angular)
