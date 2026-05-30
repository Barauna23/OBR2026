from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Stop, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import run_task, wait, StopWatch


# --- INICIALIZAÇÃO DO HARDWARE ---
hub = PrimeHub()

motor_esq = Motor(Port.A, Direction.CLOCKWISE)
motor_dir = Motor(Port.B, Direction.COUNTERCLOCKWISE)

sensor_dir = ColorSensor(Port.C)
sensor_esq = ColorSensor(Port.D)

wheel_diameter = 52
axle_track = 179

# DriveBase provê métodos de alto nível para controlar o deslocamento do robô
# (straight, turn, drive) usando as instâncias de motor e parâmetros físicos.
xambao = DriveBase(motor_esq, motor_dir, wheel_diameter, axle_track)

# -- CONSTANTES ---


reflexo_esq = sensor_esq.reflection()
reflexo_dir = sensor_dir.reflection()
hsv_dir = sensor_esq.hsv()
hsv_esq = sensor_dir.hsv()

qnt_erros = 0
souverde = False
limite_preto = 28
limite_branco = 80

h_verde_min = 155
h_verde_max = 175
s_verde_min = 75
v_verde_min = 65

kp = 2.8# proporcional
ki = 0.00025 # integral
kd = 0.21 # derivativo
integral = 0
derivative = 0
last_error = 0
erro = 0
correcao = 0

#TIVE QUE AUMENTAR O GYRO MIN E O GYRO MAX POR CAUSA DO PESO DO ROBÔ!!!!!!!!!!

GYRO_TOL = 0.02
GYRO_MIN = 85   
GYRO_MAX = 105  

VEL_MIN = 20
VEL_MAX = 50

def obstaculo():
    if UltrasonicSensor.distance() <= 100:
        print('obstaculooo')
        xambao.stop()
        GyroMove(1, 40, reverso=True
        )
def PID(kp, ki, kd):
    global integral, derivative, last_error, erro
    integral += erro
    derivative = erro - last_error
    correcao = (kp * erro) + (ki * integral) + (kd * derivative)
    last_error = erro   
    
def is_green(hsv):
    h = hsv.h
    s = hsv.s
    v = hsv.v
    return h >= h_verde_min and h <= h_verde_max and s >= s_verde_min and v >= v_verde_min


def handle_green():
    if is_green(sensor_dir.hsv()) or is_green(sensor_esq.hsv()):
        motor_dir.hold()
        motor_esq.hold()
        wait(100)

        if is_green(sensor_dir.hsv()) and is_green(sensor_esq.hsv()):
            print('verificando verde duplo')
            GyroMove(0.15, 40, reverso=True)
            if sensor_dir.reflection() <= limite_preto or sensor_esq.reflection() <= limite_preto:
                print('preto antes! ignorando')
                GyroMove(0.5, 40)
            else:
                print('sem preto atras! virando')
                GyroTurn(180)

        #================= VERDE ESQUERDO ==================================
        if is_green(sensor_esq.hsv()) and not is_green(sensor_dir.hsv()):
            print('verificar verde esquerdo')
            GyroMove(0.15, 40, reverso=True)
            if sensor_esq.reflection() <= limite_preto:
                print('preto antes do verde! ignorando', reflexo_dir, reflexo_esq)
                GyroMove(0.5, 40)
            else:
                print('sem preto atras! virando')
                GyroMove(1.5, 40)
                GyroTurn(-90)
        #================= VERDE DIREITO ==================================
        if is_green(sensor_dir.hsv()) and not is_green(sensor_esq.hsv()):
            print('verificar verde direito')
            GyroMove(0.15, 40, reverso=True)
            if sensor_dir.reflection() <= limite_preto:
                print('preto antes do verde! ignorando', reflexo_dir, reflexo_esq)
                GyroMove(0.5, 40)
            else:
                print('sem preto atras! virando')
                GyroMove(1.2, 40)
                GyroTurn(90)




def GyroMove(rotacoes, velocidade_final, reverso=False):
    global erro, correcao

    # Reset dos dois motores (importante quando usamos média)
    motor_esq.reset_angle(0)
    motor_dir.reset_angle(0)
    hub.imu.reset_heading(0)
    wait(30)

    fator_calibracao = 1

    alvo_heading = 0
    rotacoes_corrigidas = rotacoes * fator_calibracao

    # Configuração de aceleração
    velocidade_final = abs(velocidade_final)
    velocidade_atual = 40
    incremento = 2
    desacelera_a_partir = 0.8 * abs(rotacoes_corrigidas)

    sinal = -1 if reverso else 1
    corr_sinal = -1 if reverso else 1

    # Inicia movimento
    motor_esq.dc(sinal * (velocidade_atual + corr_sinal * correcao))
    motor_dir.dc(sinal * (velocidade_atual - corr_sinal * correcao))

    while True:
        # usa a média dos dois motores
        rot_atual_esq = abs(motor_esq.angle() / 360)
        rot_atual_dir = abs(motor_dir.angle() / 360)
        rot_media = (rot_atual_esq + rot_atual_dir) / 2

        if rot_media >= abs(rotacoes_corrigidas):
            break

        erro = ((alvo_heading - hub.imu.heading() + 540) % 360) - 180


def GyroTurn(graus):
    global erro, integral, derivado, correcao, last_error
    global GYRO_TOL, GYRO_MIN, GYRO_MAX

    kp = 10
    ki = 0.02
    kd = 0.025

    motor_esq.reset_angle(0)
    motor_dir.reset_angle(0)
    hub.imu.reset_heading(0)
    alvo = graus

    def erro_angular(alvo, atual):
        return ((alvo - atual + 540) % 360) - 180

    while abs(erro_angular(alvo, hub.imu.heading())) > GYRO_TOL:
        erro = erro_angular(alvo, hub.imu.heading())
        PID(kp, ki, kd)
        potencia = int(min(GYRO_MAX, max(GYRO_MIN, abs(correcao))))
        if erro > 0:
            motor_dir.dc(-potencia)
            motor_esq.dc(potencia)
        else:
            motor_dir.dc(potencia)
            motor_esq.dc(-potencia)
    xambao.stop()

    # Correção fina
    erro_residual = erro_angular(alvo, hub.imu.heading())
    if abs(erro_residual) > 0.3:
        pulso_pot = min(30, max(10, abs(int(erro_residual * 2))))
        pulso_tempo = min(100, max(40, int(abs(erro_residual) * 20)))
        if erro_residual > 0.3:
            motor_dir.dc(-pulso_pot)
            motor_esq.dc(pulso_pot)
        else:
            motor_dir.dc(pulso_pot)
            motor_esq.dc(-pulso_pot)
        wait(pulso_tempo)
        xambao.stop()

    hub.imu.reset_heading(0)


#Seguidor de linha
def seguidor(velocidade):
    global integral, derivative, last_error, erro, correcao, souverde

    integral = 0
    derivative = 0
    last_error = 0
    erro = 0
    correcao = 0

    velocidade_base_original = velocidade

    while True:

        erro = sensor_dir.reflection() - sensor_esq.reflection()

        PID(kp, ki, kd)

        velocidade_base = velocidade_base_original

        derivada_abs = abs(derivative)

        # Desaceleração progressiva em curvas
        if derivada_abs > 30:
            velocidade_base = velocidade_base_original * 0.45

        elif derivada_abs > 20:
            velocidade_base = velocidade_base_original * 0.65

        elif derivada_abs > 10:
            velocidade_base = velocidade_base_original * 0.85

        left_power = velocidade_base - correcao
        right_power = velocidade_base + correcao

        left_power = max(-100, min(100, left_power))
        right_power = max(-100, min(100, right_power))

        motor_esq.dc(int(left_power))
        motor_dir.dc(int(right_power))

        if ((is_green(sensor_dir.hsv()) or is_green(sensor_esq.hsv()))
                and souverde == False):

            handle_green()
            souverde = True

        else:
            souverde = False

        wait(5)


        

# --- MAIN LOOP ---
def main():

    seguidor(45)
    #GyroMove(2, 100)

main()