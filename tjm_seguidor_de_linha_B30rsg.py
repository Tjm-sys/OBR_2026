#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor #pyright: ignore[reportMissingImports]
from pybricks.parameters import Port, Stop, Direction, Color, Button #pyright: ignore[reportMissingImports]
from pybricks.tools import wait, StopWatch #pyright: ignore[reportMissingImports]
from pybricks.robotics import DriveBase #pyright: ignore[reportMissingImports]
from pybricks.hubs import EV3Brick #pyright: ignore[reportMissingImports]

ev3 = EV3Brick()
ev3.screen.clear()

#variaveis dos componentes
motor_direita = Motor(Port.A) 
motor_esquerda = Motor(Port.B)
motor_compartimento = Motor(Port.C)
motor_garra = Motor(Port.D)
sensor_esquerda = ColorSensor(Port.S1) # Agora o esquerdo está certo na porta S1
sensor_direita = ColorSensor(Port.S2)  # O direito está certo na porta S2
sensor_giro = GyroSensor(Port.S3)
sensor_ultrasonico = UltrasonicSensor(Port.S4)

#calibracao dos sensores #CALIBRAR
preto_esq = 5    #
branco_esq = 63  #

preto_dir = 6   #
branco_dir = 65  #

#limiares para a linha prata do resgate
limiar_prata_min = 38   # -7 do padrão
limiar_prata_max = 52   # +7 do padrão

#parametros do radar de resgate #CALIBRAR
distancia_deteccao_vitima = 250 #mm qualquer leitura menor seria vítima
distancia_pegar_vitima = 60 #mm distância da vítima em que a garra já alcança
ciclos_confirmacao_vitima = 3
potencia_giro_radar = 30
limite_varredura_radar = 380
max_vitimas_resgate = 4 
tempo_limite_resgate = 30000 #ms (30 segundos)

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
potencia = 75  #velocidade linear maxima nas retas
robot = DriveBase(motor_esquerda, motor_direita, wheel_diameter=64, axle_track=192)

#parametros do PID (kp, ki, kd)
kp = 1.8
ki = 0.05
kd = 0.5

#variaveis persistentes do controle PID e contadores de debugar verde/prata
erro_anterior = 0
integral = 0 #ref 0
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
        motor_esquerda.dc(potencia_do_giro * 1.4) #aumenta a potencia do motor ajustar caso necessario
        motor_direita.dc(-potencia_do_giro * 1.4)
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

    mover_ate_parede(40, 3000, 80) #avanca ate 3000 graus ou ate detectar a parede a 8 cm (80 mm)
    
    virar_bloco_sgiro(90, "esquerda", 40)

    mover_bloco_sgiro(40, 1800)  

#logica do resgate de radar:

def girar_para_angulo(angulo_alvo_relativo, potencia_do_giro=40, tolerancia_graus=2):

    #CALIBRAR confirmar girar pra direita (potencia positiva no robot.drive) faz o sensor_giro.angle() aumentar no robo - se for o contrario, inverter o sinal de "sentido"
    robot.stop()
    wait(200)
    
    while abs(sensor_giro.angle() - angulo_alvo_relativo) > tolerancia_graus:
        erro_giro = angulo_alvo_relativo - sensor_giro.angle()
        sentido = potencia_do_giro if erro_giro > 0 else -potencia_do_giro
        robot.drive(0, sentido)
        wait(10)
        
    robot.stop()
    wait(200)

def radar_procurar_vitima():

    log_estado("Radar")
    robot.stop()
    wait(200)
    sensor_giro.reset_angle(0)
    
    cont_deteccao = 0
    angulo_inicio_deteccao = 0
    vitima_em_vista = False
    
    while abs(sensor_giro.angle()) < limite_varredura_radar:
        distancia = sensor_ultrasonico.distance()
        
        if distancia < distancia_deteccao_vitima:
            if not vitima_em_vista:
                angulo_inicio_deteccao = sensor_giro.angle()
                vitima_em_vista = True
            cont_deteccao += 1
        else:
            if vitima_em_vista and cont_deteccao >= ciclos_confirmacao_vitima: #quando sair do pico calcula o angulo central da deteccao

                angulo_fim_deteccao = sensor_giro.angle()
                angulo_alvo = (angulo_inicio_deteccao + angulo_fim_deteccao) / 2
                robot.stop()
                log_estado("Vitima Detectada")
                return angulo_alvo
            
            cont_deteccao = 0
            vitima_em_vista = False
            
        robot.drive(0, potencia_giro_radar)
        wait(10)
        
    robot.stop()
    log_estado("Varredura Completa")
    return None

def ir_ate_vitima(angulo_alvo, potencia_aproximacao=40, max_graus_aproximacao=800):

    log_estado("Indo até vítima")
    girar_para_angulo(angulo_alvo)
    
    motor_esquerda.reset_angle(0)
    motor_direita.reset_angle(0)
    velocidade = potencia_aproximacao * 9
    motor_esquerda.run(velocidade)
    motor_direita.run(velocidade)
    
    while sensor_ultrasonico.distance() > distancia_pegar_vitima:
        if abs(motor_esquerda.angle()) >= max_graus_aproximacao:
            log_estado("Vitima nao alcancada") #por seguranca
            max_vitimas_resgate -= 1
            break
        wait(10)
        
    robot.stop()
    distancia_percorrida = abs(motor_esquerda.angle())
    wait(200)
    return distancia_percorrida

def pegar_vitima():
    log_estado("Pegando vítima")
    robot.stop()
    wait(200)
    
    motor_garra.run_angle(200, 120, wait=True) #fecha/levanta a garra #CALIBRAR
    wait(300)

def soltar_vitimas():

    log_estado("Soltando vítimas")
    robot.stop()
    wait(200)
    motor_compartimento.run_angle(200, 180, wait=True)  #abre
    wait(300)
    motor_compartimento.run_angle(200, -180, wait=True) #fecha de novo

def voltar_ao_centro(distancia_percorrida):

    log_estado("Voltando ao centro")
    
    motor_esquerda.reset_angle(0)
    motor_direita.reset_angle(0)
    velocidade = 70
    
    motor_esquerda.run_angle(velocidade, -distancia_percorrida, wait=False)
    motor_direita.run_angle(velocidade, -distancia_percorrida, wait=True)

    max_vitimas_resgate += 1
    
    robot.stop()
    wait(200)
    
    girar_para_angulo(0) #volta pro heading de referencia antes de continuar o radar

def rotina_resgate():

    #Maquina de estados: 'ENTRAR', 'RADAR', 'IR_VITIMA', 'PEGAR', 'VOLTAR_CENTRO', 'FIM'

    estado = "ENTRAR"
    vitimas_coletadas = 0
    angulo_vitima_atual = 0
    distancia_vitima_atual = 0
    cronometro_resgate = StopWatch() #inicia tempo
    
    while estado != "FIM":
        
        if estado == "ENTRAR":
            passar_resgate()
            estado = "RADAR"
            
        elif estado == "RADAR":

            if vitimas_coletadas >= max_vitimas_resgate or cronometro_resgate.time() >= tempo_limite_resgate:
                estado = "FIM"
                continue
            angulo_encontrado = radar_procurar_vitima()

            if angulo_encontrado is not None:
                angulo_vitima_atual = angulo_encontrado
                estado = "IR_VITIMA"
            else:
                estado = "FIM" #varredura completa sem achar nada novo
                
        elif estado == "IR_VITIMA":
            distancia_vitima_atual = ir_ate_vitima(angulo_vitima_atual)
            estado = "PEGAR"
            
        elif estado == "PEGAR":
            pegar_vitima()
            vitimas_coletadas += 1
            estado = "VOLTAR_CENTRO"
            
        elif estado == "VOLTAR_CENTRO":
            voltar_ao_centro(distancia_vitima_atual, angulo_vitima_atual)
            estado = "RADAR"

    #TODO: navegar ate a saida (linha/marcador) e chamar soltar_vitimas() na
    #zona de evacuacao - isso depende de como a saida e' sinalizada no campo
            
    log_estado("Resgate Finalizado")
    ev3.speaker.beep()


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
        rotina_resgate()
        erro_anterior = 0
        integral = 0
        continue

    if Button.CENTER in ev3.buttons.pressed():
        while Button.CENTER in ev3.buttons.pressed(): #segurança
            wait(10)

        robot.stop()
        log_estado("Prata B")
        ev3.speaker.beep()
        resgate_pass = 1
        wait(1000)
        rotina_resgate()
        erro_anterior = 0
        integral = 0
        continue
        
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
    elif cont_verde_dir >= 4 or cont_verde_esq >= 4:
        log_estado("V???")
        
        robot.stop()
        wait(20) #espera o chassi parar de tremer e estabilizar
        
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
        ref_sensor_esquerda = normalizar_esq(ref_esq_cru)
        ref_sensor_direita = normalizar_dir(ref_dir_cru)
   
        #calculo o erro de forma relativa (Normalização de Contraste) diminua se a luz do ambiente ou do tapete ficar mais fraca
        soma_ref = ref_sensor_esquerda + ref_sensor_direita
        if soma_ref == 0:
            soma_ref = 0.01 #evita divisão por zero se ambos lerem 0
            
        #multiplica por 70 para manter a mesma escala a no normalizar()
        ref_erro = ((ref_sensor_esquerda - ref_sensor_direita) / soma_ref) * 70
        
        #integral com limite anti-windup
        integral += ref_erro
        integral = max(min(integral, 100), -100)
        
        #calculo derivativo
        derivada = ref_erro - erro_anterior
        erro_anterior = ref_erro
        
        #formula do PID para obter a correcao angular
        correcao = (kp * ref_erro) + (ki * integral) + (kd * derivada)
        
        #VELOCIDADE DINÂMICA
        fator_bateria = ev3.battery.voltage() / 8000.0 #compensação de bateria (Bateria do EV3 carregada tem aprox. 8000mV) regulando a velocidade proporcinalmente
        potencia_max = potencia * fator_bateria
        
        reflexao_media = (ref_sensor_esquerda + ref_sensor_direita) / 2 #compensação de reflexão
        proporcao_branco = min(max(reflexao_media / 70.0, 0.0), 1.0) #garante valor entre 0 e 1
        
        #Obs: quando estiver no branco total (proporcao = 1), reduz a velocidade em 50% (0.5)
        #podemios alterar o 0.5 onde ele freie mais (ex: 0.7) ou menos (ex: 0.3)
        potencia_dinamica = potencia_max - (proporcao_branco * (potencia_max * 0.4))
        
        #drivebase com velocidade dinamica
        robot.drive(potencia_dinamica, correcao)
        wait(5)
