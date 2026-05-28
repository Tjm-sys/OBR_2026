#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, MediumMotor, MoveTank, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D # type: ignore
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4 # type: ignore
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor # type: ignore
from time import sleep

#variaveis dos componentes
motor_direita = LargeMotor(OUTPUT_A) 
motor_esquerda = LargeMotor(OUTPUT_B)
motor_garra = MediumMotor(OUTPUT_C) 
motor_conteiner = MediumMotor(OUTPUT_D)
sensor_direita = ColorSensor(INPUT_1)
sensor_esquerda = ColorSensor(INPUT_2)
sensor_giro = GyroSensor(INPUT_3)
sensor_ultrasonico = UltrasonicSensor(INPUT_4)

#Configuracoes iniciais
robot = MoveTank(OUTPUT_B, OUTPUT_A)
potencia = 80    #velocidade do robo em porcentagem nas linhas
temp_resgate = 120 #tempo programdo para fazer o resgate
vitima = 0 #vitima localizada 0/1

def mover_bloco_sgiro(potencia, num_graus_distancia): #bloco para movimento reto usando sensor
    sensor_giro.reset()    #reseta valores

    while abs(motor_esquerda.position) < num_graus_distancia:   
        diferenca_bloco_mov = sensor_giro.angle
        correcao_bloco_mov = diferenca_bloco_mov * 4.5   #Ajustar o valor "4.5" se necessario
        motor_esquerda.on(int(potencia - correcao_bloco_mov))
        motor_direita.on(int(potencia + correcao_bloco_mov))
    
    robot.off()
    sleep(0.1)

def virar_bloco_sgiro(graus_bloco_giro, direcao_bloco_giro, potencia_do_giro): #bloco para virar usando sensor
    robot.off()
    sleep(0.1)
    sensor_giro.reset()    #reseta valor
    
    if direcao_bloco_giro == "direita":
        while abs(sensor_giro.angle) < abs(graus_bloco_giro):
            robot.on(potencia_do_giro, -potencia_do_giro)
            #motor_esquerda.on(potencia_do_giro)
            #motor_direita.on(-potencia_do_giro)
            sleep(0.01)

    elif direcao_bloco_giro == "esquerda":
        while abs(sensor_giro.angle) < abs(graus_bloco_giro):
            robot.on(-potencia_do_giro, potencia_do_giro)
            #motor_esquerda.on(-potencia_do_giro)
            #motor_direita.on(potencia_do_giro)
            sleep(0.01)

    robot.off()
    sleep(0.1)

def virar_180_bloco_sgiro(graus_bloco_giro, potencia_do_giro): #bloco para movimento 180 usando sensor
    robot.off()
    sleep(0.1)
    sensor_giro.reset()    #reseta valor

    while abs(sensor_giro.angle) < abs(graus_bloco_giro):
        #robot.on(-potencia_do_giro, potencia_do_giro)
        motor_esquerda.on(potencia_do_giro)
        motor_direita.on(-potencia_do_giro)
        sleep(0.01)

    robot.off()
    sleep(0.1)

def mostrar(): #funcao para testes, mostrando os valores enquanto o codigo esta rodando
    cen_sensor_ultrasonico = sensor_ultrasonico.distance_centimeters * 10
    cor_sensor_direita = sensor_direita.color
    cor_sensor_esquerda = sensor_esquerda.color
    ref_sensor_direita = sensor_direita.reflected_light_intensity
    ref_sensor_esquerda = sensor_esquerda.reflected_light_intensity
    
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
    if sensor_ultrasonico.distance_centimeters * 10 < 120:  #distancia em mm
        caixa()
    elif sensor_direita and sensor_esquerda == "silver":
        mover_bloco_sgiro(80, 120) #*45cm
        virar_bloco_sgiro(90, "esquerda", 80)
        mover_bloco_sgiro(80, 120) #*45cm

        while temp_resgate <= 120:
            if sensor_ultrasonico.distance_centimeters * 10 >= 30 and vitima == 0:
                robot.on(-40, 40)
                dist_vitima = sensor_ultrasonico.distance_centimeters * 10
            else:
                vitima == 1
                robot.off()
                robot.on(int(40 + (dist_vitima - 5)), int(40 - (dist_vitima - 5)))
                sleep(0.05)
                motor_garra.on_for_degrees(speed=30, degrees=90) #pega vitima
                robot.on(int(40 - (dist_vitima - 5)), int(40 + (dist_vitima - 5)))
                sleep(0.05)
                vitima == 0
                
    else:
        #mostrar()
        if sensor_direita.color == ColorSensor.COLOR_RED: #and sensor_esquerda.color() != Color.GREEN:
            print("vermelho")
            motor_direita.off()
            motor_esquerda.off()

        elif sensor_direita.color == ColorSensor.COLOR_GREEN and sensor_esquerda.color == ColorSensor.COLOR_GREEN:
            print("verde nos dois")
            virar_180_bloco_sgiro(180, 80)

        elif sensor_direita.color == ColorSensor.COLOR_GREEN: #and sensor_esquerda.color() != Color.GREEN:
            sleep(0.05)
            if sensor_direita.color == ColorSensor.COLOR_GREEN: #and sensor_esquerda.color() == Color.GREEN:
                print("verde na esquerda")
                virar_bloco_sgiro(85, "esquerda", 80)

        elif sensor_esquerda.color == ColorSensor.COLOR_GREEN: #and sensor_direita.color() != Color.GREEN:
            sleep(0.05)
            if sensor_esquerda.color == ColorSensor.COLOR_GREEN: #and sensor_direita.color() == Color.GREEN:
                print("verde na direita")
                virar_bloco_sgiro(85, "direita", 80)

        else:
            sleep(0.005)
            #calculo central
            ref_sensor_direita = sensor_direita.reflected_light_intensity
            ref_sensor_esquerda = sensor_esquerda.reflected_light_intensity

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

            robot.on(int(velocidade + correcao), int(velocidade - correcao))

            #ref_erro = ref_sensor_direita - ref_sensor_esquerda #somente P
            #correcao = kp * ref_erro

            #velocidade_linear = potencia
            #velocidade_angular = correcao
            #robot.on(int(velocidade_linear + velocidade_angular), int(velocidade_linear - velocidade_angular))