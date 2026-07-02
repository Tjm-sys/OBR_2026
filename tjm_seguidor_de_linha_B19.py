#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor # pyright: ignore[reportMissingImports]
from pybricks.parameters import Port, Stop, Direction, Color # pyright: ignore[reportMissingImports]
from pybricks.tools import wait # pyright: ignore[reportMissingImports]
from pybricks.robotics import DriveBase # pyright: ignore[reportMissingImports]
from pybricks.hubs import EV3Brick # pyright: ignore[reportMissingImports]

#variaveis dos componentes
motor_direita = Motor(Port.A) 
motor_esquerda = Motor(Port.B)
sensor_direita = ColorSensor(Port.S1)
sensor_esquerda = ColorSensor(Port.S2)
sensor_giro = GyroSensor(Port.S3)
sensor_ultrasonico = UltrasonicSensor(Port.S4)

#Configurações iniciais
robot = DriveBase(motor_esquerda, motor_direita, wheel_diameter=64, axle_track=192) #ajustar conforme o robô
potencia = 40    #velocidade do robo em porcentagem nas linhas
erro_anterior = 0
integral = 0
kp = 5 #oscila muito → diminuir KP; responde lento → aumentar KP; tremendo → aumentar KD; puxando para um lado → ajustar OFFSET; erro contínuo → aumentar KI;
ki = 0.01
kd = 0.1

while True:
            
            #Seguidor de linha
            ref_sensor_direita = sensor_direita.reflection()
            ref_sensor_esquerda = sensor_esquerda.reflection()
            ref_sensores_diferenca = ref_sensor_direita - ref_sensor_esquerda
            correcao = 4.2 * ref_sensores_diferenca
    
            #Usa drive() para fazer o movimento
            velocidade_linear = potencia
            velocidade_angular = correcao
            robot.drive(velocidade_linear, velocidade_angular)
