from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, run_task, wait

# Set up all devices.
FreeFire = PrimeHub()
CorDir = ColorSensor(Port.D)
CorEsq = ColorSensor(Port.E)
corFrente = ColorSensor(Port.F)
dirmotor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
esqmotor = Motor(Port.B, Direction.CLOCKWISE)
garramotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
drive_base = DriveBase(esqmotor, dirmotor, 50, 114)

#Variaveis Globais
erro = 0
correcao = 0
integral = 0
derivado = 0
last_error = 0
P = 4.55    #4.55
I = 0.0001   #0.0002
D = 0.003   #0.001
methodup = 0
methodstop = 0
SPEED = 0
h = 0 
s = 0
v = 0
hsv_dir = 0
hsv_esq = 0
reflexao_esquerdo = 0
reflexao_direito = 0
t = 0
prata_verify = 0
veloc = 0
parede_calculo = 0
contador_colisao = 0
direita_veloc = 0
esquerda_veloc = 0
velocidade_colisão = 0




#REDEFINIR
async def redefinir():
    global last_error, integral, derivado, correcao, erro, methodstop, P, I, D
    await wait(0)
    erro = 0
    correcao = 0
    integral = 0
    derivado = 0
    last_error = 0
    P = 4.3              
    I = 0.001         
    D = 0.060
    methodstop = 0

# Fita verde
def oil_verde(hsv):
    h, s, v = hsv
    return 80 <= h <= 160 and s >= 50 and v >= 40
    
#Viu Fita Verde

async def verdedir():
    if oil_verde(await CorDir.hsv()):
        drive_base.stop()
        await wait(200)
        await GyroMoveFrente(0.32, 50)
        await wait(100)
        await GiroTurn(87)
        await wait(100)
        await GyroMoveFrente(0.35, 40)
        drive_base.stop()
        await wait(200)
        await redefinir()

async def verdeesq():
    if oil_verde(await CorEsq.hsv()):
        drive_base.stop()
        await wait(200)
        await GyroMoveFrente(0.37, 50)
        await wait(100)
        await GiroTurn(-90)
        await wait(100)
        await GyroMoveFrente(0.42, 35)
        drive_base.stop()
        await wait(200)
        await redefinir()
 


#Parede branca à frente (nao sei se uso no obstaculo)
def oil_PARABOLA_Frente(hsv):
    h, s, v = hsv
    return 343 <= h <= 350 and s <= 88 and v <= 5

#Vermelha (Fim)
def oil_vermelho(hsv):
    h, s , v = hsv
    return h <= 10 or h >= 350 and s >= 30 and v <= 60

#Triangulo Verde (Resgate)
def oil_VerdeFrente(hsv):
    h, s , v = hsv
    return 150 <= h <= 165 and s <= 90 and v <= 35

#Triangulo Vermelho (Resgate)
def oil_VermelhoFrente(hsv):
    h, s , v = hsv
    return 345 <= h <= 350 and s <= 95 and v <= 30 

#Parede (Resgate)
def oil_BrancoFrente(hsv):
    h, s , v = hsv
    return 195 <= h <= 200 and s <= 15 and v <= 90

#saídapreta
def oil_pretorescue(hsv):
    h, s , v = hsv
    return 160 <= h <= 220 and s <= 30 and v <= 80

#Fita Prata
def oil_prata(hsv):
    h, s, v = hsv
    return 204 <= h <= 208 and 19 <= s <= 24 and 66 <= v <= 71

async def teste_prauta():
    if oil_prata(await CorDir.hsv()) and oil_prata(await CorEsq.hsv()):
        wait(500)
        drive_base.stop()

  #  h1, s1, v1 = CorDir.hsv()
  #  h2, s2, v2 = CorEsq.hsv()

  #  prata_dir = v1 > 75 and s1 < 20
  #  prata_esq = v2 > 75 and s2 < 20

  #  return prata_dir or prata_esq

#Cálculo do PID
async def PID(kp, ki, kd):
    global integral, derivado, correcao, last_error, erro
    integral += erro
    derivado = erro - last_error
    correcao = kp * erro + ki * integral + kd * derivado
    last_error = erro

#SEGUIDOR DE LINHA
async def Line_Follower(velocidade):
    global SPEED, erro
    SPEED = velocidade
    while True:
        await wait(0)
        # Lê reflexos
        reflex_esq = await CorEsq.reflection()
        reflex_dir = await CorDir.reflection()
        if reflex_esq < 5 and reflex_dir < 5:
            print("Saiu da linha: ambos reflexos muito baixos.")
            drive_base.stop()
          #  break
        erro = reflex_esq - reflex_dir
        await PID(P, I, D)
        await Rampa()
        await verdedir()
        await verdeesq()
        await teste_prauta()
       # await Parabula()
      
        # Aplica velocidade corrigida
        esqmotor.dc(SPEED + correcao)

        dirmotor.dc(SPEED - correcao)

#Rampa
async def Rampa():
    global methodup

    await wait(0)

    inclinacao = FreeFire.imu.tilt()[1]

    if inclinacao <= -12 and methodup == 0:
        await drive_base.straight(60, then=Stop.COAST)
        methodup = 10
        await Garra("desce")
        drive_base.stop()
        await wait(200)

    elif inclinacao >= 0 and methodup == 10:
        methodup = 0
        await Garra("sobe")
        drive_base.stop()
        await wait(200)

#GARRA (sobe ou desce)
async def Garra(angulo):
    if angulo == "sobe":
        await garramotor.run_angle(300, -150, Stop.COAST)
    elif angulo == "desce":
        await garramotor.run_angle(300, 150, Stop.COAST)
    else:
        print("Angulo inválido. Use 'sobe' ou 'desce'.")

# Andar para FRENTE
async def GyroMoveFrente(Dist, veloc):
    global methodstop, erro
    await wait(0)
    FreeFire.imu.reset_heading(0)
    esqmotor.reset_angle(0)
    while not Dist <= methodstop:
        await wait(0)
        methodstop = esqmotor.angle() / 360
        erro = 0 - FreeFire.imu.heading()
        await PID(P,I,D)
        dirmotor.dc(veloc - correcao)

        esqmotor.dc(veloc + correcao)
        await multitask(
            wait(0),
        )
    drive_base.stop()


# Andar para TRÁS
async def GyroMoveAtras(Dist, veloc):
    global methodstop, erro
    await wait(0)
    FreeFire.imu.reset_heading(0)
    esqmotor.reset_angle(0)

    # Inverter direção se Dist for positivo (para ir para trás)
    direcao = -1 if Dist > 0 else 1
    Dist = abs(Dist)
    veloc = abs(veloc) * direcao

    while abs(methodstop) < Dist:
        await wait(0)
        methodstop = esqmotor.angle() / 360
        erro = 0 - FreeFire.imu.heading()
        await PID(P, I, D)

        # Aplicar correção normalmente
        dirmotor.dc(veloc - correcao)
        esqmotor.dc(veloc + correcao)

        await multitask(
            wait(0),
        )

    drive_base.stop()

#GIRAR (45°-90°-180°-270°-360°) (Dir=Positivo/ Esq=Negativo)
class PID_CONTROLE:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        self.integral = 0
        self.last_error = 0

    def compute(self, erro):
        self.integral += erro
        derivative = erro - self.last_error
        self.last_error = erro
        return self.P * erro + self.I * self.integral + self.D * derivative


async def GiroTurn(ref):
    global erro
    await wait(0)
    dirmotor.reset_angle(0)
    PID = PID_CONTROLE(P=1.0, I=0.01, D=0.1)

    if ref > 0:
        while FreeFire.imu.heading() <= ref:
            await wait(0)
            erro = FreeFire.imu.heading() - ref
            correcao = PID.compute(erro)
            dirmotor.dc(1 * correcao)
            esqmotor.dc(-1 * correcao)
            await multitask(wait(0))
    else:
        while FreeFire.imu.heading() >= ref:
            await wait(0)
            erro = FreeFire.imu.heading() - ref
            correcao = PID.compute(erro)
            dirmotor.dc(1 * correcao)
            esqmotor.dc(-1 * correcao)
            await multitask(wait(0))

    drive_base.stop()

#OBSTÁCULO
async def Parabula():
    await wait(0)
    if oil_PARABOLA_Frente(await corFrente.hsv()):
        await drive_base.straight(-100)
        await GiroTurn(65)
        await drive_base.straight(100)
        
        while not await CorDir.color() == Color.NONE:
            await wait(0)
            dirmotor.dc(86)

            esqmotor.dc(28)
            
            await multitask(
                wait(0), )
    drive_base.stop()

async def GyroMoveInfinity(velocfinal):
    global erro, correcao
    """
    Anda para frente infinitamente corrigindo a direção com IMU,
    até que o sensor frontal detecte verde, branco ou vermelho (HSV).
    """

    await redefinir()
    esqmotor.reset_angle(0)
    await wait(0)

    alvo_heading = FreeFire.imu.heading()

    print("GyroMoveInfinity iniciado. Alvo =", alvo_heading)

    # Configurações iniciais
    velocfinal = abs(velocfinal)
    velocidade_atual = 30
    incremento = 2
    velocidade_maxima = velocfinal
    velocidade_minima = 35

    while True:
        # Verifica HSV frontal
        hsv = await corFrente.hsv()

        # Ignorar leituras inválidas
        if hsv.h == 0 and hsv.s == 0 and hsv.v == 0:
           continue  # pula este loop e tenta de novo

        if oil_VerdeFrente(hsv) or oil_BrancoFrente(hsv) or oil_VermelhoFrente(hsv) or (
           oil_pretorescue(await CorDir.hsv()) and oil_pretorescue(await CorEsq.hsv())
        ):
           print("[✅] Cor de parada detectada:", hsv)
           break


        # Correção com PID baseado no heading inicial
        erro = ((alvo_heading - FreeFire.imu.heading() + 540) % 360) - 180
        await PID(7.5, 0.002, 0.025)

        # Aceleração progressiva
        if velocidade_atual < velocidade_maxima:
            velocidade_atual += incremento

        velocidade_atual = max(velocidade_minima, min(velocidade_atual, velocidade_maxima))
        print("heading:", FreeFire.imu.heading())

        # Aplicação nos motores (sempre para frente)
        esqmotor.dc(velocidade_atual + correcao)
        dirmotor.dc(velocidade_atual - correcao)

        await wait(10)

    # Parada suave
    esqmotor.brake()
    dirmotor.brake()

    await wait(1000)
    print("[✅] Gyro_Move_Infinito concluído. Heading final:", FreeFire.imu.heading())

    FreeFire.imu.reset_heading(0)

async def GyroMoveColisão(veloc):
    global methodstop, erro, correcao, parede_calculo, contador_colisao
    await multitask()

    while methodstop < Dist:
    
        await wait(0)
        methodstop = esqmotor.angle() / 360
        erro = 0 - FreeFire.imu.heading()
        dirmotor.dc(veloc - correcao)
        esqmotor.dc(veloc + correcao)
    
    
    Dist = 2
    await wait(0)
    FreeFire.imu.reset_heading(0)
    esqmotor.reset_angle(0)

    drive_base.stop()
    parede_calculo = False
    contador_colisao = 0

    if abs(dirmotor.speed()) > 500 or abs(esqmotor.speed()) > 500:
        parede_calculo = True
        contador_colisao = 0


if parede_calculo and (dirmotor.speed() < 400 or esqmotor.speed() < 400):
    contador_colisao += 1
    if contador_colisao >= 5:
     print("Colisão detectada! Contador:", contador_colisao)


def Mapeamentots():
    global t
    GyroMoveFrente(0.6, 60)
    GiroTurn(90)
    GyroMoveInfinity(60)
    if oil_VerdeFrente(await corFrente.hsv()) or oil_VermelhoFrente(await corFrente.hsv()):
        elif oil_VerdeFrente(await corFrente.hsv()):
            t = 1
            GiroTurn(-45)
            GyroMoveFrente(0.6, 60)
            GiroTurn(-45)
    else oil_BrancoFrente(await corFrente.hsv()):
        await GiroTurn(-90)
        await GyroMoveColisão(60)

    if oil_VerdeFrente(await corFrente.hsv()) or oil_VermelhoFrente(await corFrente.hsv()):
        elif oil_VerdeFrente(await corFrente.hsv()):
            t = 2
            GiroTurn(-45)
            GyroMoveFrente(0.6, 60)
            GiroTurn(-45)
    else oil_BrancoFrente(await corFrente.hsv()):
        await GiroTurn(-90)
        await GyroMoveColisão(60)

    if oil_VerdeFrente(await corFrente.hsv()) or oil_VermelhoFrente(await corFrente.hsv()):
        elif oil_VerdeFrente(await corFrente.hsv()):
            t = 3
            GiroTurn(-45)
            GyroMoveFrente(0.6, 60)
            GiroTurn(-45)
    else oil_BrancoFrente(await corFrente.hsv()):
        await GiroTurn(-90)
        await GyroMoveColisão(60)
    if t = 0
         t= 4
           await GiroTurn(-135)
           await GyroMoveFrente(1.6, 60)
           await GiroTurn(-45)
        FreeFire.imu.reset_heading(0)
    

def cv1():
    await Garra("desce")
    await GyroMoveColisão


#direita_veloc = dirmotor.speed
#esquerda_veloc = esqmotor.speed

async def main():
   # await Line_Follower(60)
    await GyroMoveColisão(60)
 

run_task(main())