#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor #pyright: ignore[reportMissingImports]
from pybricks.parameters import Port, Stop, Direction, Color #pyright: ignore[reportMissingImports]
from pybricks.tools import wait #pyright: ignore[reportMissingImports]
from pybricks.robotics import DriveBase #pyright: ignore[reportMissingImports]
from pybricks.hubs import EV3Brick #pyright: ignore[reportMissingImports]

ev3 = EV3Brick()
ev3.screen.clear()

#variaveis dos componentes
motor_direita = Motor(Port.A) 
motor_esquerda = Motor(Port.B)
sensor_direita = ColorSensor(Port.S1) #fisicamente na Esquerda devido a inversao dos cabos no robo
sensor_esquerda = ColorSensor(Port.S2) #fisicamente na Direita devido a inversao dos cabos no robo
sensor_giro = GyroSensor(Port.S3)
sensor_ultrasonico = UltrasonicSensor(Port.S4)

#calibracao dos sensores
preto_esq = 8
branco_esq = 80

preto_dir = 6.5
branco_dir = 70

def normalizar_esq(valor):
    denominador = branco_esq - preto_esq
    if denominador <= 0:
        denominador = 1
    return max(min((valor - preto_esq) * 70 / denominador, 70), 0)

def normalizar_dir(valor):
    denominador = branco_dir - preto_dir
    if denominador <= 0:
        denominador = 1
    return max(min((valor - preto_dir) * 70 / denominador, 70), 0)

#parametros de velocidade e controle
potencia = 60         #velocidade linear maxima nas retas
velocidade_min = 35   #velocidade linear minima nas curvas para estabilidade
robot = DriveBase(motor_esquerda, motor_direita, wheel_diameter=64, axle_track=192)

#parametros do PID (kp, ki, kd)
kp = 1.8
ki = 0.01
kd = 0.5

#variaveis persistentes do controle PID e contadores de debugar verde
erro_anterior = 0
integral = 10 #ref 0
cont_verde_dir = 0
cont_verde_esq = 0

#controle de estado para depuracao sem sobrecarregar a tela do EV3
estado_atual = None

def log_estado(novo_estado):
    global estado_atual
    if novo_estado != estado_atual:
        estado_atual = novo_estado
        print("[DEBUG]:", novo_estado)
        ev3.screen.print(novo_estado)

def mover_bloco_sgiro(potencia_motor, num_graus_distancia):
    log_estado("Reto")
    robot.stop() #para o DriveBase para liberar o controle direto dos motores
    motor_esquerda.reset_angle(0)
    motor_direita.reset_angle(0)
    
    #converte potencia (0-100) para velocidade regulada em graus por segundo (max ~900 deg/s)
    velocidade = abs(potencia_motor) * 9
    #o sinal da potencia determina se o robo anda para frente ou para tras
    angulo = num_graus_distancia if potencia_motor >= 0 else -num_graus_distancia
    
    #roda os dois motores com PID interno sincronizado por encoders
    motor_esquerda.run_angle(velocidade, angulo, wait=False)
    motor_direita.run_angle(velocidade, angulo, wait=True)
    
    robot.stop()
    wait(200)

def virar_bloco_sgiro(graus_bloco_giro, direcao_bloco_giro, potencia_do_giro):
    log_estado("Girando {} graus".format(graus_bloco_giro))
    robot.stop()
    wait(200)
    sensor_giro.reset_angle(0)
    
    if direcao_bloco_giro == "direita":
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            robot.drive(0, potencia_do_giro)
            wait(10)
    elif direcao_bloco_giro == "esquerda":
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            robot.drive(0, -potencia_do_giro)
            wait(10)
            
    robot.stop()
    wait(200)

def virar_bloco_sgiro_verde(graus_bloco_giro, direcao_bloco_giro, potencia_do_giro):
    log_estado("Giro Verde")
    robot.stop()
    wait(200)
    sensor_giro.reset_angle(0)
    
    limite_preto_linha = 15 #limite de refletividade normalizada para considerar linha preta encontrada
    
    if direcao_bloco_giro == "direita":
        #se girando para a direita, o sensor fisico da direita (sensor_esquerda) eh o primeiro a tocar a linha
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            ref_dir = normalizar_dir(sensor_esquerda.reflection())
            if ref_dir < limite_preto_linha:
                log_estado("Linha Dir Ok")
                break
            robot.drive(0, potencia_do_giro)
            wait(10)
            
    elif direcao_bloco_giro == "esquerda":
        #se girando para a esquerda, o sensor fisico da esquerda (sensor_direita) eh o primeiro a tocar a linha
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            ref_esq = normalizar_esq(sensor_direita.reflection())
            if ref_esq < limite_preto_linha:
                log_estado("Linha Esq Ok")
                break
            robot.drive(0, -potencia_do_giro)
            wait(10)
            
    robot.stop()
    wait(200)

def virar_180_bloco_sgiro(graus_bloco_giro, potencia_do_giro):
    log_estado("Girando 180 graus")
    robot.stop()
    wait(100)
    sensor_giro.reset_angle(0)
    
    while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
        motor_esquerda.dc(potencia_do_giro)
        motor_direita.dc(-potencia_do_giro)
        wait(10)
        
    robot.stop()
    wait(100)

def desviar_obstaculo():
    log_estado("Obstaculo")
    
    virar_bloco_sgiro(90, "direita", 40)

    mover_bloco_sgiro(40, 500) 

    virar_bloco_sgiro(90, "esquerda", 40)

    mover_bloco_sgiro(40, 1200) 
    
    virar_bloco_sgiro(90, "esquerda", 40)

    mover_bloco_sgiro(40, 600)
    
    virar_bloco_sgiro(90, "direita", 40)

#inicializa o bloco EV3 e limpa a tela
log_estado("Iniciando robo")
wait(800)

while True:
    if sensor_ultrasonico.distance() < 80:
        desviar_obstaculo()
        erro_anterior = 0
        integral = 0
        continue
        
    cor_dir = sensor_direita.color()
    cor_esq = sensor_esquerda.color()
    
    #converte leitura consecutiva de verde
    if cor_dir == Color.GREEN:
        cont_verde_dir += 1
    else:
        cont_verde_dir = 0
        
    if cor_esq == Color.GREEN:
        cont_verde_esq += 1
    else:
        cont_verde_esq = 0
    
    #detecao de Linha Vermelha
    if cor_dir == Color.RED and cor_esq == Color.RED:
        robot.stop()
        log_estado("Vermelho")
        break
        
    #detecao de Verde cruzamentos/intersecoes
    elif cont_verde_dir >= 3 or cont_verde_esq >= 3:
        log_estado("V???")
        
        #le as cores e as reflexoes novamente
        cor_dir_confirm = sensor_direita.color()
        cor_esq_confirm = sensor_esquerda.color()
        ref_dir = sensor_direita.reflection()
        ref_esq = sensor_esquerda.reflection()
        
        #filtro de reflexao
        verde_dir = (cor_dir_confirm == Color.GREEN) and (4 <= ref_dir <= 9)
        verde_esq = (cor_esq_confirm == Color.GREEN) and (4 <= ref_esq <= 9)
        
        if verde_dir and verde_esq:
            wait(10)
            if verde_dir and verde_esq:
                log_estado("Verde Duplo 180")
                virar_180_bloco_sgiro(180, 40)
            
        elif verde_dir:
            wait(10)
            if verde_dir:
                log_estado("Verde Direita")
                robot.drive(300, 0) # avanca reto usando robot.drive para evitar conflitos de controle
                wait(500)
                robot.stop()
                virar_bloco_sgiro_verde(90, "esquerda", 40)
            
        elif verde_esq:
            wait(10)
            if verde_esq:
                log_estado("Verde Esquerda")
                robot.drive(300, 0) # avanca reto usando robot.drive para evitar conflitos de controle
                wait(500)
                robot.stop()
                virar_bloco_sgiro_verde(90, "direita", 40)
            
        cont_verde_dir = 0
        cont_verde_esq = 0
        erro_anterior = 0
        integral = 0
        continue
        
    else:
        log_estado("Seguindo Linha")
        #sensor_direita esta fisicamente na Esquerda
        ref_sensor_direita = normalizar_esq(sensor_direita.reflection())
        #sensor_esquerda esta fisicamente na Direita
        ref_sensor_esquerda = normalizar_dir(sensor_esquerda.reflection())
        
        #se o robo estiver no branco puro (gap), zera a integral para nao puxar para o lado
        if ref_sensor_direita > 55 and ref_sensor_esquerda > 55:
            integral = 0
        
        #calcula o erro (Diferenca de reflexao)
        ref_erro = ref_sensor_direita - ref_sensor_esquerda
        
        #integral com limite anti-windup
        integral += ref_erro
        integral = max(min(integral, 100), -100)
        
        #calculo derivativo
        derivada = ref_erro - erro_anterior
        erro_anterior = ref_erro
        
        #formula do PID para obter a correcao angular
        correcao = (kp * ref_erro) + (ki * integral) + (kd * derivada)
        
        #driveBase com velocidade constante (B22)
        robot.drive(potencia, correcao)
        wait(5)
