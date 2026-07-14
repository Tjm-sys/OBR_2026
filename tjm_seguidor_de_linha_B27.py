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
sensor_esquerda = ColorSensor(Port.S1) # Agora o esquerdo está certo na porta S1
sensor_direita = ColorSensor(Port.S2)  # O direito está certo na porta S2
sensor_giro = GyroSensor(Port.S3)
sensor_ultrasonico = UltrasonicSensor(Port.S4)

#calibracao dos sensores #CALIBRAR
preto_esq = 6    #
branco_esq = 75  #

preto_dir = 4   #
branco_dir = 60  #

#limiares para a linha prata do resgate (refletividade cinza de ~45% medida no tapete)
limiar_prata_min = 38   # -7 do padrão
limiar_prata_max = 52   # +7 do padrão

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
potencia = 60  #velocidade linear maxima nas retas
robot = DriveBase(motor_esquerda, motor_direita, wheel_diameter=64, axle_track=192)

#parametros do PID (kp, ki, kd)
kp = 1.8
ki = 0.05
kd = 0.5

#variaveis persistentes do controle PID e contadores de debugar verde/prata
erro_anterior = 0
integral = 10 #ref 0
cont_verde_dir = 0
cont_verde_esq = 0
cont_prata = 0 #contador de ciclos consecutivos lendo prata
resgate_pass = 0

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
        #se girando para a direita, o sensor da direita eh o primeiro a tocar a linha
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            ref_dir = normalizar_dir(sensor_direita.reflection())
            if ref_dir < limite_preto_linha:
                log_estado("Linha Dir Ok")
                break
            robot.drive(0, potencia_do_giro)
            wait(10)
            
    elif direcao_bloco_giro == "esquerda":
        #se girando para a esquerda, o sensor da esquerda eh o primeiro a tocar a linha
        while abs(sensor_giro.angle()) < abs(graus_bloco_giro):
            ref_esq = normalizar_esq(sensor_esquerda.reflection())
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

def mover_ate_parede(potencia_motor, num_graus_max, limite_distancia_mm):
    log_estado("Parede")
    robot.stop()
    motor_esquerda.reset_angle(0)
    motor_direita.reset_angle(0)
    
    #converte potencia (0-100) para velocidade regulada em graus por segundo
    velocidade = abs(potencia_motor) * 9
    if potencia_motor < 0:
        velocidade = -velocidade
        
    motor_esquerda.run(velocidade)
    motor_direita.run(velocidade)
    
    #monitora ate que a distancia limite seja atingida ou o limite de graus seja estourado
    while abs(motor_esquerda.angle()) < abs(num_graus_max):
        dist = sensor_ultrasonico.distance()
        if dist < limite_distancia_mm:
            log_estado("Parede Ok")
            break
        wait(10)
        
    robot.stop()
    wait(200)

def desviar_obstaculo():
    log_estado("Obstaculo")
    
    virar_bloco_sgiro(90, "direita", 40)

    mover_bloco_sgiro(40, 500) 

    virar_bloco_sgiro(90, "esquerda", 40)

    mover_bloco_sgiro(40, 1200) 
    
    virar_bloco_sgiro(90, "esquerda", 40)

    mover_bloco_sgiro(40, 500)
    
    virar_bloco_sgiro(90, "direita", 40)

def passar_resgate():
    log_estado("Resgate")

    mover_bloco_sgiro(40, 1900) 

    virar_bloco_sgiro(90, "direita", 40)

    #avanca ate 3000 graus ou ate detectar a parede a 8 cm (80 mm)
    mover_ate_parede(40, 3000, 80) 
    
    virar_bloco_sgiro(90, "esquerda", 40)

    mover_bloco_sgiro(40, 1800)
    
#inicializa o bloco EV3 e limpa a tela
log_estado("Iniciando robo")
wait(800)

while True:
    if sensor_ultrasonico.distance() < 80: #and resgate_pass == 1
        desviar_obstaculo()
        erro_anterior = 0
        integral = 0
        continue
        
    cor_dir = sensor_direita.color()
    cor_esq = sensor_esquerda.color()
    ref_dir_cru = sensor_direita.reflection()
    ref_esq_cru = sensor_esquerda.reflection()
    
    '''
    #atualiza contadores de cor prata (reflexao cinza intermediaria de 45%)
    if ((limiar_prata_min <= ref_dir_cru <= limiar_prata_max) and 
        (limiar_prata_min <= ref_esq_cru <= limiar_prata_max) and 
        (cor_dir not in (Color.GREEN, Color.RED) and cor_esq not in (Color.GREEN, Color.RED))):
        cont_prata += 1
    else:
        cont_prata = 0
        
    #detecao de linha prata (entrada na area de resgate)
    if cont_prata >= 5:
        robot.stop()
        log_estado("Prata")
        ev3.speaker.beep()
        resgate_pass = 1
        wait(1000)
        passar_resgate()
    '''
        
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
        ev3.speaker.beep()
        break
        
    #detecao de Verde cruzamentos/intersecoes
    elif cont_verde_dir >= 3 or cont_verde_esq >= 3:
        log_estado("V???")
        
        # DICA DE PRECISÃO: Parar o robô ANTES de fazer a leitura de confirmação!
        # Se você ler com o robô andando, ele pode já ter passado do verde.
        robot.stop()
        wait(50) # Espera o chassi parar de tremer e estabilizar
        
        # Agora sim, le as cores e as reflexoes novamente com o robô cravado no chão
        cor_dir_confirm = sensor_direita.color()
        cor_esq_confirm = sensor_esquerda.color()
        ref_dir = sensor_direita.reflection()
        ref_esq = sensor_esquerda.reflection()
        
        #filtro de reflexao
        verde_dir = (cor_dir_confirm == Color.GREEN) and (4 <= ref_dir <= 10)
        verde_esq = (cor_esq_confirm == Color.GREEN) and (4 <= ref_esq <= 10)
        if verde_dir and verde_esq:
            log_estado("Verde Duplo 180")
            virar_180_bloco_sgiro(175, 40)
        
        #agora que as portas estao arrumadas, verde na direita vira para a direita
        elif verde_dir:
             log_estado("Verde Direita")
             robot.drive(300, 0) # avanca reto usando robot.drive para evitar conflitos de controle
             wait(500)
             robot.stop()
             virar_bloco_sgiro_verde(90, "direita", 40) # Consertado para virar à direita

        elif verde_esq:
            log_estado("Verde Esquerda")
            robot.drive(300, 0) # avanca reto usando robot.drive para evitar conflitos de controle
            wait(500)
            robot.stop()
            virar_bloco_sgiro_verde(90, "esquerda", 40) # Consertado para virar à esquerda
            
        cont_verde_dir = 0
        cont_verde_esq = 0
        erro_anterior = 0
        integral = 0
        continue
        
    else:
        log_estado("Seguindo Linha")
        # Agora as portas estão corretas, então chamamos a normalização correta e limpa
        ref_sensor_esquerda = normalizar_esq(ref_esq_cru)
        ref_sensor_direita = normalizar_dir(ref_dir_cru)
        
        #se o robo estiver no branco puro (gap), zera a integral para nao puxar para o lado
        if ref_sensor_direita > 55 and ref_sensor_esquerda > 55:
            integral = 0
        
        # Calcula o erro de forma relativa (Normalização de Contraste)
        # Isso garante que a intensidade da curva não diminua se a luz do ambiente ou do tapete ficar mais fraca.
        # Usa a fórmula (Esquerda - Direita) / (Esquerda + Direita)
        soma_ref = ref_sensor_esquerda + ref_sensor_direita
        if soma_ref == 0:
            soma_ref = 0.1 # Evita divisão por zero se ambos lerem 0
            
        # Multiplicamos por 70 para manter a mesma escala que você já usava no normalizar() (que vai até 70)
        # Assim você não precisa refazer o "kp" do zero!
        ref_erro = ((ref_sensor_esquerda - ref_sensor_direita) / soma_ref) * 70
        
        #integral com limite anti-windup
        integral += ref_erro
        integral = max(min(integral, 100), -100)
        
        #calculo derivativo
        derivada = ref_erro - erro_anterior
        erro_anterior = ref_erro
        
        #formula do PID para obter a correcao angular
        correcao = (kp * ref_erro) + (ki * integral) + (kd * derivada)
        
        #driveBase com velocidade constante
        robot.drive(potencia, correcao)
        wait(5)