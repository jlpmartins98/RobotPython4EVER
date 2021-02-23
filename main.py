#!/usr/bin/env python3


# from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank, MediumMotor,SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from time import sleep

from ev3dev2.wheel import EV3Tire
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_C, SpeedPercent, MoveDifferential, SpeedRPM, MediumMotor, OUTPUT_B
from random import randint
import copy as cp  # preciso para fazer o algoritmo sena o python altera a variave global

cs = ColorSensor(INPUT_4)


class Pastor:
    def __init__(self, posicao, direcao):
        self.posicao = posicao  # quadrado em que se encontra
        # lado para o qual esta virado (0 graus é para cima)
        self.direcao = direcao


class cacifo:
    def __init__(self, parentCacifo, numeroCacifo, custoCaminho, custoTotal):
        # o numero do quadrado (para o robo saber a posiçao)
        self.numeroCacifo = numeroCacifo
        self.distanciaOjetivo = 0  # a heuristica, distancia até a cerca/ovelha
        self.custoCaminho = custoCaminho  # o G na funçao do A*
        self.custoTotal = custoTotal  # O f na funçao do A*
        self.paredeUp = False
        self.paredeDown = False
        self.paredeRight = False
        self.paredeLeft = False
        self.parentCacifo = parentCacifo  # de onde veio (so é usado no A*)
        # self.movementDebuff = 0         #custo adicional qd encontra uma parede(para n ir para elas)


array_pode_avancar = []
# ev3=EV3Brick()
precionado = 0
encontrou_parede = 0
batatadas_totais = 0

##################MOTORES#########################
# motor_esquerdo = Motor(OUTPUT_A)
# motor_direito = Motor(OUTPUT_C)
# motor_braco = MediumMotor(OUTPUT_B,Direction.COUNTERCLOCKWISE)
toque = TouchSensor(INPUT_1)
i = 0
obstacle_sensor = UltrasonicSensor(INPUT_2)
# sensor_toque = TouchSensor(Port.S1)
# sensor_cor = ColorSensor(INPUT_4)
# robot= MoveTank(OUTPUT_A, OUTPUT_C)
# robot = DriveBase(motor_esquerdo, motor_direito, wheel_diameter = 55.5, axle_track= 104)
#####################################################
STUD_MM = 8
robot = MoveDifferential(OUTPUT_A, OUTPUT_C, EV3Tire, 9 * STUD_MM)
braco = MediumMotor(OUTPUT_B)

graus = [90, 180, 270]
lugares_visitados = []
cacifos_visitados = [1]
cacifos_prioritarios = []
posicao_ovelhas = []
Sound = Sound()


informacao = Pastor(1, 0)  # Informação sobre o robot
# arrayCacifos_com_heuristica = [cacifo(1,10,0,11,False,False,False,False)] #array que guarda os cacifos com o seu numero e heuristica ate á cerca

# array que guarda os cacifos com o seu numero e heuristica ate á cerca
arrayCacifos_com_heuristica = [cacifo(None, 1, 0, 11)]


def guarda_posicao_ovelha():
    if(len(posicao_ovelhas) != 2):
        k = informacao.posicao
        if(informacao.direcao == 0):  # se a ovelha estiver acima do robo
            k += 6
        if(informacao.direcao == 90):  # se a ovelha estiver a esquerda
            k -= 1
        if(informacao.direcao == 180):  # se a ovelha estiver abaixo
            k -= 6
        if(informacao.posicao == 270):  # se a ovelha estiver a direita
            k += 1
        if(k not in posicao_ovelhas):
            posicao_ovelhas.append(k)


def inicializaCacifos():  # funcao que da os valores da heuristica e custo do caminho a todos os cacifos (antes de encontrar paredes/ovelhas)
    k = 9  # k é a heuristica do cacifo até a cerca
    for j in range(2, 37):  # j é o numero do cacifo
        if(j == 7):
            k = 9
        if(j == 13):
            k = 8
        if(j == 19):
            k = 7
        if(j == 25):
            k = 6
        if(j == 31):
            k = 5
        CacifoClasse = cacifo(None, j, 0, k)
        arrayCacifos_com_heuristica.append(CacifoClasse)
        k -= 1


def CacifoAtual(numero_cacifo):
    # caso esteja a procura dum cacifo fora do tabuleiro
    cacifoFiller = cacifo(None, 100, 200, 300)
    for j in arrayCacifos_com_heuristica:
        if(j.numeroCacifo == numero_cacifo):
            return j
    return cacifoFiller  # quando procura algo fora dos limites do tabuleiro


# devolve um array com o caminho(nº dos cacifos a seguir começando na sua posiçao)
def algoritmo_A_star(goal):
    global arrayCacifos_com_heuristica
    # faz uma copia da cacifoComParedeiavel
    arrayBackup = cp.deepcopy(arrayCacifos_com_heuristica)
    openList = []  # cacifos que nao verificou
    closedList = []  # cacifos que ja verificou
    inicio = CacifoAtual(informacao.posicao)
    inicio.custoTotal = 0  # isto modifica a cacifoComParedeivel global mas pq?
    openList.append(inicio)
    while(len(openList) > 0):
        # vai buscar o cacifo atual
        # restora o array ao estado original para nao criar parentNodes infinitos
        arrayCacifos_com_heuristica = cp.deepcopy(arrayBackup)
        currentNode = openList[0]
        currentIndex = 0
        # item é o elemento atual da lista; index é o index do elemento atual
        for index, item in enumerate(openList):
            # caso exista um cacifo mais barato do que o currentNode
            if(item.custoTotal < currentNode.custoTotal):
                currentNode = item
                currentIndex = index
        # tira o cacifo verificado da openList e mete na lista dos que ja verificou
        openList.pop(currentIndex)
        closedList.append(currentNode)
        # mudar o current node para o mais barato
        if(currentNode.numeroCacifo == goal):  # caso chegue ao objetivo
            path = []
            current = currentNode
            # ciclo para guardar o caminho, vai percorrendo os pais dos nos ate chegar ao inicio
            while(current is not None):
                path.append(current.numeroCacifo)
                current = current.parentCacifo
            # devolve o caminho ao contrario (ou seja na ordem começando onde ele esta)
            return path[::-1]
        # caso esteja num dos limites
        children = []
        childUp = CacifoAtual(37)
        childDown = CacifoAtual(37)
        childLeft = CacifoAtual(37)
        childRight = CacifoAtual(37)
        # gera os cacifos adjacentes
        if(currentNode.numeroCacifo <= 30):
            # abre o cacifo em cima do currentNode
            childUp = CacifoAtual(currentNode.numeroCacifo + 6)
            childUp.parentCacifo = currentNode
        if(currentNode.numeroCacifo > 6):
            # abre o cacifo abaixo do currentNode
            childDown = CacifoAtual(currentNode.numeroCacifo - 6)
            childDown.parentCacifo = currentNode
        if(currentNode.numeroCacifo not in [31, 25, 19, 13, 7, 1]):
            # abre o cacifo a esquerda
            childLeft = CacifoAtual(currentNode.numeroCacifo - 1)
            childLeft.parentCacifo = currentNode
        if(currentNode.numeroCacifo not in [36, 30, 24, 18, 12, 6]):
            # abre o cacifo a direita
            childRight = CacifoAtual(currentNode.numeroCacifo + 1)
            childRight.parentCacifo = currentNode
        # verificaçoes para saber se tem paredes a volta/esta nos limites do tabuleiro
        # caso tenha uma parede em cima ou esteja nos cacifos do topo
        if(currentNode.paredeUp == True or currentNode.numeroCacifo >= 31):
            childUp.custoCaminho += 100
        # caso parede em baixo ou esteja na primeira linha de cacifos
        if(currentNode.paredeDown == True or currentNode.numeroCacifo < 7):
            childDown.custoCaminho += 100
        # caso parede a esquerda ou esteja nos cacifos mais a esquerda
        if(currentNode.paredeLeft == True or currentNode.numeroCacifo in [31, 25, 19, 13, 7, 1]):
            childLeft.custoCaminho += 100
        # caso parede a direita ou esteja nos cacifos mais a direta
        if(currentNode.paredeRight == True or currentNode.numeroCacifo in [36, 30, 24, 18, 12, 6]):
            childRight.custoCaminho += 100
        children.append(childUp)
        children.append(childDown)
        children.append(childLeft)
        children.append(childRight)
        for child in children:
            if(child in closedList):  # se ja verificou este child
                continue
            # custa sempre 1 para andar para qq um dos child pois sao os cacifos adajacentes
            child.custoCaminho += currentNode.custoCaminho + 1
            if(child.numeroCacifo < goal):  # calculo da heuristica
                child.distanciaOjetivo = goal - child.numeroCacifo
            if(child.numeroCacifo >= goal):  # calculo da heuristica
                child.distanciaOjetivo = child.numeroCacifo - goal
            # custo total para chegar a este cacifo
            child.custoTotal = child.custoCaminho + child.distanciaOjetivo
            if(child in openList):  # caso nao tenha verificado este child
                var = openList.index(child)
                # verifica se o custoCaminho do child atual é maior do que o proximo elemento na openList (ou seja se é o mais barato) se nao for passa po proximo
                if(child.custoCaminho >= openList[var].custoCaminho):
                    continue
            # adiciona a child á lista dos nao verificados
            openList.append(child)


def adiciona_parede(cacifoComParede):
    j = 0
    while(j != len(arrayCacifos_com_heuristica)):
        if(arrayCacifos_com_heuristica[j].numeroCacifo == cacifoComParede):
            if(informacao.direcao == 0):  # Virado para cima
                # guarda que este cacifo tem uma parede em cima
                arrayCacifos_com_heuristica[j].paredeUp = True
                # colocamos a parede do cacifo acima
                if(arrayCacifos_com_heuristica[j].numeroCacifo < 31):
                    # guarda que o cacifo de cima do atual tem uma parede em baixo
                    arrayCacifos_com_heuristica[j+6].paredeDown = True

            elif(informacao.direcao == 270):  # Virado para a direita
                # guarda que este cacifo tem uma parede a dirieta
                arrayCacifos_com_heuristica[j].paredeRight = True
                # colocamos a parede do cacifo a sua direita
                # caso nao esteja nos limites do mapa a direita
                if(arrayCacifos_com_heuristica[j].numeroCacifo not in [36, 30, 24, 18, 12, 6]):
                    # guarda que o cacifo a direita do atual tem uma parde a sua esquerda
                    arrayCacifos_com_heuristica[j+1].paredeLeft = True

            elif(informacao.direcao == 180):  # Virado para baixo
                # guarda que este cacifo tem uma parede em baixo
                arrayCacifos_com_heuristica[j].paredeDown = True
                # colocamos a parede no cacifo abaixo
                # caso nao esteja nos limites do mapa em baixo
                if(arrayCacifos_com_heuristica[j].numeroCacifo > 6):
                    # guarda que o cacifo a abaixo do atual tem uma parede em cima
                    arrayCacifos_com_heuristica[j-6].paredeUp = True

            elif(informacao.direcao == 90):  # Virado para a esquerda
                arrayCacifos_com_heuristica[j].paredeLeft = True
                # colocamos a parede do cacifo a esquerda
                # caso nao esteja nos limites do mapa a esquerda
                if(arrayCacifos_com_heuristica[j].numeroCacifo not in [1, 7, 13, 19, 25, 31]):
                    # guarda que o cacifo a direita do atual tem uma parde a sua esquerda
                    arrayCacifos_com_heuristica[j-1].paredeRight = True
        j += 1


# adiciona o cacifo, ao array cacifos visitados
def adiciona_visitados(pos):
    visitado = procura_visitado(pos)  # procura se já está no array
    if(visitado == False):
        cacifos_visitados.append(pos)
    print(cacifos_visitados)

# percorre o array dos visitados


def procura_visitado(t):
    for j in cacifos_visitados:
        if (j == t):  # e ve se o cacifo já está lá
            return True
    return False


def verifica_cacifo():
    global i
    global array_pode_avancar  # Array que guarda as direções que o robot pode ir
    global cacifos_prioritarios
    while(cs.color == 6):  # Avança até ao limite do cacifo
        ################robot.drive(75,-1)####################
        robot.on(30, 30)  # suposto andar ate receber a instrução off
        ##################################################
    robot.on(0, 0)
    if(cs.color == 5):  # Encontrou parede RED
        ovelhas()
        adiciona_parede(informacao.posicao)  # Adiciona parede ao array
        Sound.beep()
        sleep(0.5)
        # robot.straight(-50)
        robot.on_for_distance(SpeedRPM(-40), 50)  # Volta para trás
        sleep(0.5)
        i += 1  # Atualiza o i
        vira(90)
    elif(cs.color == 2 or cs.color == 1):  # Encontrou limite do cacifo BLACK ####tá azul agora era (1)
        ovelhas()
        if(obstacle_sensor.distance_centimeters < 20):  # verificar distancia
            robot.on_for_distance(SpeedRPM(-40), 50)
            sleep(0.5)
            # vira(90)
        ######################robot.straight(-50)#####################
        # robot.on_for_rotations(-20, -20, 0.5)    #Volta para trás, suposto voltar ate ao centro do cacifo
        ###############################################################
        else:
            robot.on_for_distance(SpeedRPM(-40), 50)
            sleep(0.5)
            # vira(90)
        # Verifica se pode avançar(Se não é limite do tabuleiro)
        teste_pode_avancar = pode_avancar()
        i += 1  # Atualiza o i
        if(teste_pode_avancar == True):  # Caso possa avançar nessa direção
            # Adiciona essa direção ao array
            array_pode_avancar.append(informacao.direcao)
        vira(90)

    if(i >= 4):  # Já verificou todos os lados do cacifo
        # Dos arrays possíveis procura um que não foi visitado
        escolhe_prioridade(array_pode_avancar)
        # Obtem tamanho do array prioritario
        opcoes_prioridade = len(cacifos_prioritarios)
        print(cacifos_prioritarios)
        print(array_pode_avancar)
        if(opcoes_prioridade > 0):  # Se existir algum com prioridade
            op_prio = opcoes_prioridade - 1
            if(op_prio > 0):
                # Escolhe um aleatóriamente
                aleatorio_prio = randint(0, op_prio)
                direcao_prioridade = cacifos_prioritarios[aleatorio_prio]
                coloca_direcao(direcao_prioridade)
            else:
                coloca_direcao(cacifos_prioritarios[0])
        else:
            # numero de opcoes disponiveis
            opcoes = len(array_pode_avancar) - 1
            if(opcoes > 0):
                aleatorio = randint(0, opcoes)  # Escolhe uma aleatoriamente
                direcao = array_pode_avancar[aleatorio]
                coloca_direcao(direcao)  # coloca o robot nesse direção
            else:
                coloca_direcao(array_pode_avancar[0])
        ############robot.straight(200) #Avança até o próximo cacifo######################
        # robot.on_for_rotations(20, 20, 1) # suposto andar ate ao proximo cacifo
        robot.on_for_distance(SpeedRPM(20), 150)
        ####################################################################################
        adiciona_visitados(informacao.posicao)
        atualiza_posicao()
        i = 0  # Reseta o contador
        array_pode_avancar = []  # Limpa o array
        cacifos_prioritarios = []  # Limpa prioritários

# muda a direcao


def coloca_direcao(direcao):
    while(informacao.direcao != direcao):   # Roda para a esquerda até a direção ser a pretendida
        vira(90)

# escolhe qual cacifo ir


def escolhe_prioridade(lista):
    for k in lista:
        if (k == 0):
            if(procura_visitado(informacao.posicao + 6) == False):  # verifica o cacifo de cima
                cacifos_prioritarios.append(k)
        if (k == 90):
            if(procura_visitado(informacao.posicao - 1) == False):  # verifica o cacifo da esquerda
                cacifos_prioritarios.append(k)
        if (k == 180):
            if(procura_visitado(informacao.posicao - 6) == False):  # verifica o cacifo de baixo
                cacifos_prioritarios.append(k)
        if (k == 270):
            if(procura_visitado(informacao.posicao + 1) == False):  # verifica o cacifo da direita
                cacifos_prioritarios.append(k)


def ovelhas():  # quando encontra ovelhas
    # encontrar maneira de ele saber quando gritar e quando dar porrada
    global batatadas_totais  # total de vezes que ja bateu nas ovelhas
    #aleatorio = randint(0,2)
    global precionado
    # ev3.speaker.beep()
    if(obstacle_sensor.distance_centimeters < 20):  # verificar distancia
        Sound.beep()

        sleep(1)
        braco.on_for_degrees(100,360)
        #if(toque.is_pressed):
        sleep(1)
        #braco.stop()
        #volta com o braço para cima 
        braco.on_for_degrees(100,-360)
        if(len(posicao_ovelhas) != 2): #se ainda nao tiver encontrado as duas ovelhas guarda a posiçao da q encontrou
            guarda_posicao_ovelha()
        #else:
            ############## Mudar isto ###############
            # Já nao é preciso bater na ovelha
            # if(aleatorio == 0): #bate na ovelha
        #while(precionado == 0):
       
            # precionado=0
            # elif(aleatorio == 1): #ladra com a ovelha
            # ev3.speaker.play_file(SoundFile.DOG_BARK_2)
        sleep(2)


def vira(graus):  # vira o robo pelos graus inseridos (sentido contrahorario)
    # robot.turn(graus)
    # robot.turn_degrees( SpeedPercent(5), graus )
    robot.turn_right(SpeedRPM(20), graus)

    if(graus == 270):  # para endireitar o robo como ele nunca vira direito
        robot.turn_right(SpeedRPM(20), -20)
    elif(graus == 180):  # para endireitar o robo como ele nunca vira direito
       robot.turn_right(SpeedRPM(20), -20)
    elif(graus == 90):  # para endireitar o robo como ele nunca vira direito
       robot.turn_right(SpeedRPM(20), -15)
    informacao.direcao = informacao.direcao + graus  # atualiza a direçao do robo
    # da o resto da divisao inteira da sua direçao por 360 para os graus nunca ultrapassarem 360
    informacao.direcao = int(informacao.direcao % 360)


def pode_avancar():
    if(informacao.direcao == 0):  # Se estiver virado para cima
        if(informacao.posicao > 30):  # E na linha de cima
            return False  # não pode avançar
        else:
            return True  # caso contrario avança
    elif(informacao.direcao == 270):  # Se estiver virado para a direita
        if(informacao.posicao % 6 == 0):  # e na coluna da direita
            return False  # não pode avançar
        else:
            return True  # caso contrario avança
    elif(informacao.direcao == 180):  # Se estiver virado para baixo
        if(informacao.posicao < 7):  # e na linha de baixo
            return False  # não pode avançar
        else:
            return True  # caso contrario avança
    else:  # Se estiver virado para a esquerda
        if(informacao.posicao % 6 == 1):  # e na coluna da esquerda
            return False  # não pode avançar
        else:
            return True  # caso contrario avança


def atualiza_posicao():
    if(informacao.direcao == 0):  # Virado para cima
        informacao.posicao = informacao.posicao + 6
    elif(informacao.direcao == 270):  # Virado para a direita
        informacao.posicao = informacao.posicao + 1
    elif(informacao.direcao == 180):  # Virado para baixo
        informacao.posicao = informacao.posicao - 6
    elif(informacao.direcao == 90):  # Virado para a esquerda
        informacao.posicao = informacao.posicao - 1

# cacifo(self,parentCacifo,numeroCacifo,custoCaminho,custoTotal) + up
# arrayCacifos_com_heuristica = [cacifo(1,10,0,11,False,False,False,False)] #array que guarda os cacifos com o seu numero e heuristica ate á cerca
# paredeDown, paredeLeft, paredeRight, paredeUp


def guia_ovelha(pathovelha):
    j = 0
    while(j < len(pathovelha)):
        # if(arrayCacifos_com_heuristica[j+1].numeroCacifo in 1,31,6): #isto já não está resolvido com as paredes? (acho que não porque se não houver paredes ele vai quere sempre usar o braço quando a ovelha está nestas posições)
        # funcao braco()
        # estes whiles e k's desta parte sei lá é random stuff x) acho que pode ser que seja útil para fazer mover 2 casas mas talvez não
        # k = 0
        # k = j+2
        # while(j <k)
        # andar 2 casas
        #  j+=1
        #if(informacao.posicao+6 == posicao_ovelhas[1] or informacao.posicao-6 == posicao_ovelhas[1] or informacao.posicao-1 == posicao_ovelhas[1] or informacao.posicao+1 == posicao_ovelhas[1]):
            # funcao braco()
            # andar 2 casas
            #j += 2

        # Para paredes em L e/ou 3 paredes num cacifo (limites também fazem parte da parede) (caso andar para cima)
        if(pathovelha[j].numeroCacifo+6 == pathovelha[j+1].numeroCacifo):
            if((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeRight == True) or (pathovelha[j+1].paredeUp == True and pathovelha[j+1] in [36, 30, 24, 18, 12, 6]) or (pathovelha[j+1] > 30 and pathovelha[j+1].paredeRight == True)):  # tirar o 36 da lista?
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(0)
                robot.on_for_distance(SpeedRPM(20), 150)
                coloca_direcao(90)
                robot.on_for_distance(SpeedRPM(20), 150)
                j+= 2
            
            elif((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeLeft == True) or (pathovelha[j+1].paredeUp == True and pathovelha[j+1] in [31,25,19,13,7,1]) or (pathovelha[j+1] > 30 and pathovelha[j+1].paredeLeft == True) ):
                # braco
                # andar 2 casas fazendo l para a esquerda (cima e direita) FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(0) 
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(270)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            
            else:
                # apitar e andar 1 casa FEITO
                Sound.beep() 
                coloca_direcao(0)
                robot.on_for_distance(SpeedRPM(20),150)
                j+= 1
            
        elif(pathovelha[j].numeroCacifo+1 == pathovelha[j+1].numeroCacifo):#Para o caso de querer andar para a direita 
            if((pathovelha[j+1].paredeRight == True and pathovelha[j+1].paredeUp == True) or (pathovelha[j+1].paredeRight == True and pathovelha[j+1] > 30 ) or (pathovelha[j+1] in [36,30,24,18,12,6] and pathovelha[j+1].paredeUp == True)):#tirar o 36 da lista?
                # braco
                # andar 2 casas fazendo l para a direita ( direita e depois baixo) FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(270)
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(180)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            elif((pathovelha[j+1].paredeRight == True and pathovelha[j+1].paredeDown == True) or (pathovelha[j+1].paredeRight == True and pathovelha[j+1] < 7) or (pathovelha[j+1] in [36,30,24,18,12,6] and pathovelha[j+1].paredeDown == True)): #tirar o 36 da lista
                # braco
                # andar 2 casas fazendo l para a direita ( direita e depois cima) FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(270)
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(0)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            
            else:
                # apitar e andar 1 casa FEITO
                Sound.beep() 
                coloca_direcao(270)
                robot.on_for_distance(SpeedRPM(20),150)
                j+= 1
            
        
        elif(pathovelha[j].numeroCacifo-6 == pathovelha[j+1].numeroCacifo): #para o caso de querer andar para baixo
            if((pathovelha[j+1].paredeDown == True and pathovelha[j+1].paredeRight == True) or (pathovelha[j+1].paredeDown == True and pathovelha[j+1] in [36,30,24,18,12,6]) or (pathovelha[j+1] < 7 and pathovelha[j+1].paredeRight == True)):
                # braco
                # andar 2 casas fazendo l para a esquerda (baixo e esquerda) #FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(180)
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(90)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            elif((pathovelha[j+1].paredeDown == True and pathovelha[j+1].paredeLeft == True) or (pathovelha[j+1].paredeDown == True and pathovelha[j+1] in [31,25,19,13,7,1]) or (pathovelha[j+1] < 7 and pathovelha[j+1].paredeLeft == True)):
                # braco
                # andar 2 casas fazendo l para a direita (baixo e direita) FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(180)
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(270)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            
            else:
                # apitar 
                # e andar 1 casa FEITO
                Sound.beep() 
                coloca_direcao(180)
                robot.on_for_distance(SpeedRPM(20),150)
                j+= 1
            
        elif(pathovelha[j].numeroCacifo-1 == pathovelha[j+1].numeroCacifo): #para o caso de querer andar para a esquerda 
            if((pathovelha[j+1].paredeLeft == True and pathovelha[j+1].paredeDown == True) or (pathovelha[j+1].paredeLeft == True and pathovelha[j+1] < 6) or (pathovelha[j+1] in [31,25,19,13,7,1] and pathovelha[j+1].paredeDown == True)):
                # braco
                # andar 2 casas fazendo l ( esquerda e depois cima) FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(90)
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(0)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            elif((pathovelha[j+1].paredeLeft == True and pathovelha[j+1].paredeUp == True) or (pathovelha[j+1].paredeLeft == True and pathovelha[j+1] > 30) or (pathovelha[j+1] in [31,25,19,13,7,1] and pathovelha[j+1].paredeUp == True)):
                # braco
                # andar 2 casas fazendo l ( esquerda e depois baixo) FEITO
                sleep(1)
                braco.on_for_degrees(100,360)
                #if(toque.is_pressed):
                sleep(1)
                #braco.stop()
                #volta com o braço para cima 
                braco.on_for_degrees(100,-360)
                coloca_direcao(90)
                robot.on_for_distance(SpeedRPM(20),150)
                coloca_direcao(180)
                robot.on_for_distance(SpeedRPM(20),150) 
                j+= 2
            
            else:
                # apitar 
                # e andar 1 casa FEITO
                Sound.beep() 
                coloca_direcao(90)
                robot.on_for_distance(SpeedRPM(20),150)
                j+= 1
            
        
        
                 


# m = LargeMotor(OUTPUT_A)
# verifica o tabuleiro todo
# chama o A* para o melhor caminho para a ovelha 

def main():
    # inicializaCacifos()
    inicializaCacifos()
    #k = algoritmo_A_star(30)
    # robot.on_for_position(10, 90)
    # verifica_cacifo()
    # m.on_for_rotations(SpeedPercent(50),10)
    
    while True:
        verifica_cacifo()
        # if(sensor_cor.color() == Color.BLUE):
            # robot.Stop()
        if(informacao.posicao == 36):
            break
        #verificou o tabuleiro todo
    algoritmo_A_star(posicao_ovelhas[0])    
    # print(k)

if (__name__ == "__main__"):
    main() 

            



