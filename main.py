#!/usr/bin/env python3


# from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank, MediumMotor,SpeedPercent
from math import inf
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from time import sleep
from sys import stderr
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


array_pode_avancar = []
ovelhas_na_cerca = 0
precionado = 0
encontrou_parede = 0
batatadas_totais = 0
paredes_encontradas = 0
paredes = []
toque = TouchSensor(INPUT_1)
i = 0
obstacle_sensor = UltrasonicSensor(INPUT_2)
STUD_MM = 8
robot = MoveDifferential(OUTPUT_A, OUTPUT_C, EV3Tire, 9 * STUD_MM)
braco = MediumMotor(OUTPUT_B)

graus = [90, 180, 270]
lugares_visitados = []
cacifos_visitados = [1]
cacifos_prioritarios = []
cacifos_adjacentes = []
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
        if(informacao.direcao == 270):  # se a ovelha estiver a direita
            k += 1
        if(k not in posicao_ovelhas): # Só adiciona caso a ovelha não seja repetida
            posicao_ovelhas.append(k)
            cacifos_visitados.append(k)


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
def algoritmo_A_star(goal,ignoreSheep):
    global arrayCacifos_com_heuristica
    # faz uma copia da cacifoComParedeiavel
    arrayBackup = cp.deepcopy(arrayCacifos_com_heuristica)
    openList = []  # cacifos que nao verificou
    closedList = []  # cacifos que ja verificou
    inicio = CacifoAtual(informacao.posicao)
    inicio.custoTotal = 0 
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
        closedList.append(currentNode.numeroCacifo)
        # mudar o current node para o mais barato
        if(currentNode.numeroCacifo == goal):  # caso chegue ao objetivo
            path = []
            current = currentNode
            # ciclo para guardar o caminho, vai percorrendo os pais dos nos ate chegar ao inicio
            while(current is not None):
                path.append(current)
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
            if(child.numeroCacifo in closedList):  # se ja verificou este child
                continue
            # custa sempre 1 para andar para qq um dos child pois sao os cacifos adajacentes
            child.custoCaminho += currentNode.custoCaminho + 1
            if(child.numeroCacifo in posicao_ovelhas and ignoreSheep == False):
                child.custoCaminho += 100
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
    global paredes_encontradas
    AlreadyFound = True
    j = 0
    while(j != len(arrayCacifos_com_heuristica)):
        if(arrayCacifos_com_heuristica[j].numeroCacifo == cacifoComParede):
            if(informacao.direcao == 0):  # Virado para cima
                if(arrayCacifos_com_heuristica[j].paredeUp != True):
                    AlreadyFound = False
                # guarda que este cacifo tem uma parede em cima
                arrayCacifos_com_heuristica[j].paredeUp = True
                # colocamos a parede do cacifo acima
                if(arrayCacifos_com_heuristica[j].numeroCacifo < 31):
                    if(arrayCacifos_com_heuristica[j+6].paredeDown != True):
                        AlreadyFound = False
                    # guarda que o cacifo de cima do atual tem uma parede em baixo
                    arrayCacifos_com_heuristica[j+6].paredeDown = True

            elif(informacao.direcao == 270):  # Virado para a direita
                if(arrayCacifos_com_heuristica[j].paredeRight != True):
                    AlreadyFound = False
                # guarda que este cacifo tem uma parede a dirieta
                arrayCacifos_com_heuristica[j].paredeRight = True
                # colocamos a parede do cacifo a sua direita
                # caso nao esteja nos limites do mapa a direita
                if(arrayCacifos_com_heuristica[j].numeroCacifo not in [36, 30, 24, 18, 12, 6]):
                    if(arrayCacifos_com_heuristica[j+1].paredeLeft != True):
                        AlreadyFound = False
                    # guarda que o cacifo a direita do atual tem uma parde a sua esquerda
                    arrayCacifos_com_heuristica[j+1].paredeLeft = True

            elif(informacao.direcao == 180):  # Virado para baixo
                if(arrayCacifos_com_heuristica[j].paredeDown != True):
                    AlreadyFound = False
                # guarda que este cacifo tem uma parede em baixo
                arrayCacifos_com_heuristica[j].paredeDown = True
                # colocamos a parede no cacifo abaixo
                # caso nao esteja nos limites do mapa em baixo
                if(arrayCacifos_com_heuristica[j].numeroCacifo > 6):
                    if(arrayCacifos_com_heuristica[j-6].paredeUp != True):
                        AlreadyFound = False
                    # guarda que o cacifo a abaixo do atual tem uma parede em cima
                    arrayCacifos_com_heuristica[j-6].paredeUp = True

            elif(informacao.direcao == 90):  # Virado para a esquerda
                if(arrayCacifos_com_heuristica[j].paredeLeft != True):
                    AlreadyFound = False
                arrayCacifos_com_heuristica[j].paredeLeft = True
                # colocamos a parede do cacifo a esquerda
                # caso nao esteja nos limites do mapa a esquerda
                if(arrayCacifos_com_heuristica[j].numeroCacifo not in [1, 7, 13, 19, 25, 31]):
                    if(arrayCacifos_com_heuristica[j-1].paredeRight != True):
                        AlreadyFound = False
                    # guarda que o cacifo a direita do atual tem uma parde a sua esquerda
                    arrayCacifos_com_heuristica[j-1].paredeRight = True
        j += 1
    if(AlreadyFound == False):
        paredes_encontradas+=1


# adiciona o cacifo, ao array cacifos visitados
def adiciona_visitados(pos):
    visitado = procura_visitado(pos)  # procura se já está no array
    if(visitado == False):
        cacifos_visitados.append(pos)

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
    global cacifos_adjacentes
    while(cs.color == 6):  # Avança até ao limite do cacifo
        robot.on(20, 20)  # suposto andar ate receber a instrução off
    robot.on(0, 0)
    if(cs.color == 5):  # Encontrou parede RED
        ovelhas()
        adiciona_parede(informacao.posicao)  # Adiciona parede ao array
        sleep(0.5)
        robot.on_for_distance(SpeedRPM(-20), 50)  # Volta para trás
        sleep(0.5)
        i += 1  # Atualiza o i
        vira(90)
    elif(cs.color == 2 or cs.color == 1 or cs.color == 4):  # Encontrou limite do cacifo BLACK ####tá azul agora era (1)
        #ovelhas()
        if(obstacle_sensor.distance_centimeters < 20):  # verificar distancia
            sleep(2)
            if(len(posicao_ovelhas) != 2): #se ainda nao tiver encontrado as duas ovelhas guarda a posiçao da q encontrou
                guarda_posicao_ovelha()
            robot.on_for_distance(SpeedRPM(-20), 50)
            sleep(0.5)
            i += 1

        else:
            robot.on_for_distance(SpeedRPM(-20), 50)
            sleep(0.5)
            teste_pode_avancar = pode_avancar()
            teste_ovelha= verifica_ovelha() #Caso falhe o sensor a reconhecer a ovelha
            i += 1  # Atualiza o i
            
            print(teste_ovelha,file= stderr)
            print("teste_pode_avancar",file= stderr)
            print(teste_pode_avancar,file= stderr)
            if(teste_pode_avancar == True and teste_ovelha== True):  # Caso possa avançar nessa direção
                # Adiciona essa direção ao array
                array_pode_avancar.append(informacao.direcao)
        # Verifica se pode avançar(Se não é limite do tabuleiro)
        
        vira(90)

    if(i >= 4):  # Já verificou todos os lados do cacifo
        escolhe_prioridade(array_pode_avancar) #Verifica se nos arrays adjacentes há algum não visitado, caso exista, esse cacifo passa a prioritário
        escolhe_adjacentes(array_pode_avancar)
        # Obtem tamanho do array prioritario
        opcoes_prioridade = len(cacifos_prioritarios)
        opcoes_adjacentes = len(cacifos_adjacentes)
        if(opcoes_prioridade > 0):  # Se existir algum com prioridade (Cacifos exatamente ao lado que não tenham sido visitados)
            op_prio = opcoes_prioridade - 1
            if(op_prio > 0):
                # Escolhe um aleatóriamente
                aleatorio_prio = randint(0, op_prio)
                direcao_prioridade = cacifos_prioritarios[aleatorio_prio]
                coloca_direcao(direcao_prioridade)
            else:
                coloca_direcao(cacifos_prioritarios[0])
            
        elif(opcoes_adjacentes > 0): # Caso não exista um exatamente ao lado, verifica os adjacentes a esses
            op_adjacente = opcoes_adjacentes -1
            if(op_adjacente>0):
                aleatorio_adjacente = randint(0, op_adjacente)
                direcao_adjacentes = cacifos_adjacentes[aleatorio_adjacente]
                coloca_direcao(direcao_adjacentes)
            else:
                coloca_direcao(cacifos_adjacentes[0])

        else: # Caso todos os cacifos exatamente ao lado, e os adjancentes já tenham sido verificados, escolhe um aleatório dos possíveis a avançar
            opcoes = len(array_pode_avancar) - 1
            if(opcoes > 0):
                aleatorio = randint(0, opcoes)  # Escolhe uma aleatoriamente
                direcao = array_pode_avancar[aleatorio]
                coloca_direcao(direcao)  # coloca o robot nesse direção
            else:
                coloca_direcao(array_pode_avancar[0])
            

        robot.on_for_distance(SpeedRPM(20), 150)
        adiciona_visitados(informacao.posicao)
        atualiza_posicao()
        i = 0  # Reseta o contador
        array_pode_avancar = []  # Limpa o array
        cacifos_prioritarios = []  # Limpa prioritários
        cacifos_adjacentes = [] # Limpa adjacentes

# muda a direcao

def verifica_ovelha():
    if(len(posicao_ovelhas)==1):
        if(informacao.direcao == 0):  # Verifica se tem ovelha no cacifo a cima
            if(informacao.posicao +6 == posicao_ovelhas[0]):  # E na linha de cima
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
        elif(informacao.direcao == 270):  # Verifica se tem ovelha no cacifo a direita
            if(informacao.posicao +1 == posicao_ovelhas[0]):  # e na coluna da direita
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
        elif(informacao.direcao == 180):  # Verifica se tem ovelha no cacifo a baixo
            if(informacao.posicao -6 == posicao_ovelhas[0]):  # e na linha de baixo
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
        else:  # Verifica se tem ovelha no cacifo a esquerda
            if(informacao.posicao -1 == posicao_ovelhas[0]):  # e na coluna da esquerda
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
    elif(len(posicao_ovelhas)==2):
        if(informacao.direcao == 0):  # Verifica se tem ovelha no cacifo a cima
            if(informacao.posicao +6 == posicao_ovelhas[0] or informacao.posicao+6 == posicao_ovelhas[1]):  # E na linha de cima
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
        elif(informacao.direcao == 270):  # Verifica se tem ovelha no cacifo a direita
            if(informacao.posicao +1 == posicao_ovelhas[0] or informacao.posicao+1 == posicao_ovelhas[1]):  # e na coluna da direita
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
        elif(informacao.direcao == 180):  # Verifica se tem ovelha no cacifo a baixo
            if(informacao.posicao -6 == posicao_ovelhas[0] or informacao.posicao-6 == posicao_ovelhas[1]):  # e na linha de baixo
                return False  # não pode avançar
            else:
                return True  # caso contrario avança
        else:  # Verifica se tem ovelha no cacifo a esquerda
            if(informacao.posicao -1 == posicao_ovelhas[0] or informacao.posicao-1 == posicao_ovelhas[1]):  # e na coluna da esquerda
                return False  # não pode avançar
            else:
                return True  # caso contrario avança 
    else:
        return True  

def escolhe_adjacentes(lista):
    for k in lista:
        if (k==0):
            if(informacao.posicao+12 not in cacifos_visitados and informacao.posicao+12<36 and CacifoAtual(informacao.posicao+6).paredeUp == False):
                cacifos_adjacentes.append(k)
        if (k==180):
            if(informacao.posicao-12 not in cacifos_visitados and informacao.posicao-12>0 and CacifoAtual(informacao.posicao-6).paredeDown == False):
                cacifos_adjacentes.append(k)
        if (k==270):
            if(informacao.posicao+2 not in cacifos_visitados and informacao.posicao+2<36 and CacifoAtual(informacao.posicao+1).paredeRight == False):
                if(informacao.posicao not in [5,6,11,12,17,18,23,24,29,30,35,36]):
                    cacifos_adjacentes.append(k)
        if (k==90):
            if(informacao.posicao-2 not in cacifos_visitados and informacao.posicao-2>0 and CacifoAtual(informacao.posicao-1).paredeLeft == False):
                if(informacao.posicao not in [1,2,7,8,13,14,19,20,25,26,31,32]):
                    cacifos_adjacentes.append(k)

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
    if(obstacle_sensor.distance_centimeters < 20):  # verificar distancia
        if(len(posicao_ovelhas) != 2): #se ainda nao tiver encontrado as duas ovelhas guarda a posiçao da q encontrou
            guarda_posicao_ovelha()
        sleep(2)


def vira(graus):  # vira o robo pelos graus inseridos (sentido contrahorario)

    robot.turn_right(SpeedRPM(20), graus)

    if(graus == 270):  # para endireitar o robo como ele nunca vira direito
        robot.turn_right(SpeedRPM(20), -20) # Tabuleiro da "mesa"
        #robot.turn_right(SpeedRPM(20), -30)
    elif(graus == 180):  # para endireitar o robo como ele nunca vira direito
       robot.turn_right(SpeedRPM(20), -20) # Tabuleiro da "mesa"
       #robot.turn_right(SpeedRPM(20), -30)
    elif(graus == 90):  # para endireitar o robo como ele nunca vira direito
       robot.turn_right(SpeedRPM(20), -15) # Tabuleiro da "mesa"
       #robot.turn_right(SpeedRPM(20), -20)
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

def atualiza_ovelha(o,pos):
    if (o == posicao_ovelhas[0]):
        posicao_ovelhas[0] += pos
    else:
        posicao_ovelhas[1] += pos
    print("atualiza",file= stderr)
    print(posicao_ovelhas[0],file= stderr)
    print(posicao_ovelhas[1],file= stderr)
def guia_ovelha(ovelha):
    global ovelhas_na_cerca
    if (ovelha == posicao_ovelhas[0]):
        ovelha2 = posicao_ovelhas[1]
    else:
        ovelha2 = posicao_ovelhas[0]
    cacifo_ovelha = CacifoAtual(ovelha)
     #devolve o cacifo onde está a ovelha
    #atualiza_ovelha(ovelha,ondefoi) #usado para atualizar a posição da ovelha após interagirmos com ela
    while(cacifo_ovelha.numeroCacifo < 36): #Enquanto a ovelha não chegar à cerca
        print("posicao ovelha",file= stderr)
        print(ovelha,file= stderr)
        print(cacifo_ovelha.numeroCacifo,file= stderr)
        #Novo Código
        cacifo_ovelha = CacifoAtual(ovelha)
        
        if(cacifo_ovelha.numeroCacifo+6 == ovelha2 or cacifo_ovelha.numeroCacifo+1 == ovelha2 or cacifo_ovelha.numeroCacifo-6 == ovelha2 or cacifo_ovelha.numeroCacifo-1 == ovelha2 ):
            #3 paredes
            cacifo_cima = CacifoAtual(ovelha+6)
            cacifo_baixo = CacifoAtual(ovelha-6)
            cacifo_direita = CacifoAtual(ovelha+1)
            cacifo_esquerda = CacifoAtual(ovelha-1)
            if(cacifo_ovelha.paredeUp and cacifo_ovelha.paredeDown and (cacifo_ovelha.paredeLeft or cacifo_ovelha.paredeRight)):
                if (cacifo_ovelha.paredeLeft):
                    coloca_direcao(0) #virado para cima
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                    ovelha = ovelha+1
                    coloca_direcao(270) #virado para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                else:
                    coloca_direcao(0) #virado para cima
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                    ovelha = ovelha-1
                    coloca_direcao(90) #virado para a esquerda
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
            elif(cacifo_ovelha.paredeRight and cacifo_ovelha.paredeLeft and (cacifo_ovelha.paredeDown or cacifo_ovelha.paredeUp)):
                if(cacifo_ovelha.paredeDown):
                    coloca_direcao(270) #virado para a direita
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,6) #ovelha vai para cima
                    ovelha = ovelha+6
                    coloca_direcao(0) #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                else:
                    coloca_direcao(270) #virado para a direita
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,-6) #ovelha vai para baixo
                    ovelha = ovelha-6
                    coloca_direcao(90) #virado para baixo
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao-=6 #atualiza a posição do robot a dizer que foi para baixa
            #casos de paredes em L   
            
            if(cacifo_ovelha.paredeDown and cacifo_ovelha.paredeRight):
                if(cacifo_cima.paredeRight):
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,12) #ovelha vai para a cima 2 vezes
                    ovelha = ovelha+12
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                else:
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,7) #ovelha vai para a cima e depois para a direita
                    ovelha = ovelha+7
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    coloca_direcao(0) #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para a direita

            elif(cacifo_ovelha.paredeLeft and cacifo_ovelha.paredeUp):
                if(cacifo_direita.paredeUp):
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,2) #ovelha vai para a direita 2 vezes
                    ovelha = ovelha+2
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    coloca_direcao(270) #virado para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                else:
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,7) #ovelha vai para a direita 2 vezes
                    ovelha = ovelha+7
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    coloca_direcao(270) #virado para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita

            elif(cacifo_ovelha.paredeUp and cacifo_ovelha.paredeRight):
                if(cacifo_esquerda.paredeUp):
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,-2) #ovelha vai para a esquerda 2 vezes
                    ovelha = ovelha-2
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    
                    coloca_direcao(90) #virado para a esquerda
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
                else:
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,5) #ovelha vai para a esquerda e para cima
                    ovelha = ovelha+5
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    coloca_direcao(90) #virado para a esquerda
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda

            elif(cacifo_ovelha.paredeLeft and cacifo_ovelha.paredeDown):
                if(cacifo_cima.paredeLeft):
                    coloca_direcao(90) #virado para a esquerda
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,12) #ovelha vai para a cima 2 vezes
                    ovelha = ovelha+12
                    informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
                    coloca_direcao(0)   #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                else:
                    coloca_direcao(90) #virado para a esquerda
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,5) #ovelha vai para a cima e depois para a esquerda
                    ovelha = ovelha+5
                    informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
                    coloca_direcao(0) #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima

            

                    
            #casos de 1 parede ou limites
            elif(cacifo_ovelha.paredeUp or cacifo_ovelha.paredeRight or cacifo_ovelha.paredeLeft or cacifo_ovelha.paredeDown):
                if(cacifo_ovelha.paredeUp):
                    if(cacifo_direita.paredeRight):
                        sleep(1)
                        coloca_direcao(270) #virado para direita
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,7) #ovelha vai para a direita e para cima
                        ovelha = ovelha+7
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    else:
                        sleep(1)
                        coloca_direcao(270) #virado para direita
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,2) #ovelha vai para a direita 2 vezes
                        ovelha = ovelha+2
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                
                elif(cacifo_ovelha.paredeRight):
                    if(cacifo_cima.paredeUp):
                        sleep(1)
                        coloca_direcao(0) #virado para cima
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,7) #ovelha vai para cima e para a direita
                        ovelha = ovelha+7
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                    else:
                        sleep(1)
                        coloca_direcao(0) #virado para cima
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,12) #ovelha vai para cima 2 vezes
                        ovelha = ovelha+12
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                elif(cacifo_ovelha.paredeDown):
                    if(cacifo_direita.paredeRight):
                        sleep(1)
                        coloca_direcao(270) #virado para direita
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,7) #ovelha vai para a direita e para cima
                        ovelha = ovelha+7
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    else:
                        sleep(1)
                        coloca_direcao(270) #virado para direita
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,2) #ovelha vai para a direita 2 vezes
                        ovelha = ovelha+2
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                elif(cacifo_ovelha.paredeLeft):
                    if(cacifo_cima.paredeUp):
                        sleep(1)
                        coloca_direcao(0) #virado para cima
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,7) #ovelha vai para cima e para a direita
                        ovelha = ovelha+7
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                    else:
                        sleep(1)
                        coloca_direcao(0) #virado para cima
                        braco.on_for_degrees    (100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        atualiza_ovelha(ovelha,12) #ovelha vai para cima 2 vezes
                        ovelha = ovelha+12
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                        robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                elif(cacifo_ovelha.numeroCacifo in [31]):
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    atualiza_ovelha(ovelha,2) #ovelha vai 2 para a direita
                    ovelha = ovelha+2
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    coloca_direcao(270) #virado para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                elif(cacifo_ovelha.numeroCacifo in [6]):
                    #Usa o braço
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    atualiza_ovelha(ovelha,12) #ovelha vai 2 para cima
                    ovelha = ovelha+12
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    coloca_direcao(0) #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                elif(cacifo_ovelha.numeroCacifo in [1]):
                    #Para o caso da ovelha estar no cacifo 1
                    if(informacao.posicao == 2): #robot no cacifo 2
                        coloca_direcao(90)
                        braco.on_for_degrees(100,360)                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        atualiza_ovelha(ovelha,12) #ovelha vai 2 para cima
                        ovelha = ovelha+12
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
                        coloca_direcao(0) #virado para cima
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                    else: #robot no cacifo 7
                        coloca_direcao(180)
                        braco.on_for_degrees(100,360)                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        atualiza_ovelha(ovelha,2) #ovelha vai 2 para a direita
                        ovelha = ovelha+2
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao-=6 #atualiza a posição do robot a dizer que desceu
                        coloca_direcao(270) #virado para a direita
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
            
            
            elif(cacifo_ovelha.numeroCacifo in [32,33,34,35]): #caso de estar nos cacifos do topo do tabuleiro
                if(cacifo_direita.paredeRight):
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,-5) #ovelha vai para a direita e para baixo
                    ovelha = ovelha-5
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                else:
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,2) #ovelha vai para a direita 2 vezes
                    ovelha = ovelha+2
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
            elif(cacifo_ovelha.numeroCacifo in [2,3,4,5]): #estar nos cacifos da parte de baixo do tabuleiro
                if(cacifo_direita.paredeRight):
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,-5) #ovelha vai para a direita e para baixo
                    ovelha = ovelha-5
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                else:
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,2) #ovelha vai para a direita 2 vezes
                    ovelha = ovelha+2
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
            elif (cacifo_ovelha.numeroCacifo in [30,24,18,12]): #estar no cacifos mais à direita
                if(cacifo_cima.paredeUp):
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,5) #ovelha vai para cima e esquerda
                    ovelha = ovelha+5
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                else:
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,12) #ovelha vai para cima 2 vezes
                    ovelha = ovelha+12
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima

            elif (cacifo_ovelha.numeroCacifo in [26,27,28,29,20,21,22,23,14,15,16,17,8,9,10,11] and cacifo_ovelha.paredeDown == False and cacifo_ovelha.paredeLeft == False and cacifo_ovelha.paredeUp == False and cacifo_ovelha.paredeRight == False): #Caso de estar num cacifo sem paredes e que não faça parte dos limites do tabuleiro
                if(cacifo_cima.paredeUp and cacifo_cima.paredeRight): 
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,5) #ovelha vai para cima e a esquerda
                    ovelha = ovelha+5
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                elif(cacifo_cima.paredeUp):
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,7) #ovelha vai para cima e a direita
                    ovelha = ovelha+7
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                else:
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360)                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360)
                    atualiza_ovelha(ovelha,12) #ovelha vai para cima 2 vezes
                    ovelha = ovelha+12
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cim
            
            #fim dos casos com as 2 ovelhas adjacentes
            else:
                j = 0
                pathovelha = algoritmo_A_star(36,True) 
                if(pathovelha[j].numeroCacifo+6 == pathovelha[j+1].numeroCacifo): #Para o caso de querer subir 
                    if((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeRight == True)): 
                        sleep(1)
                        sleep(1)
                        coloca_direcao(0)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=6
                        j+= 1
                    
                    elif((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeLeft == True)):
                        sleep(1)
                        sleep(1)
                        coloca_direcao(0) 
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=6
                        j+= 1
                    elif ((pathovelha[j+1].paredeUp == True and pathovelha[j+1].numeroCacifo in [36, 30, 24, 18, 12, 6]) or (pathovelha[j+1].numeroCacifo > 30 and pathovelha[j+1].paredeRight == True)):              
                        sleep(1)
                    
                        coloca_direcao(0)
                        braco.on_for_degrees(100,360)                
                        sleep(1) 
                        braco.on_for_degrees(100,-360)
                        #volta com o braço para cim
                        robot.on_for_distance(SpeedRPM(40),200)
                        coloca_direcao(90)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=6
                        informacao.posicao-=1
                        j+= 2
                    elif ((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeLeft == True) or (pathovelha[j+1].paredeUp == True and pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]) or (pathovelha[j+1].numeroCacifo > 30 and pathovelha[j+1].paredeLeft == True)):
                        sleep(1)
                        
                        coloca_direcao(0)
                        braco.on_for_degrees(100,360)                
                        sleep(1) 
                        #volta com o braço para cim
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(40),200)
                        coloca_direcao(270)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=6
                        informacao.posicao+=1
                        j+= 2
                    else:
                        Sound.beep() 
                        coloca_direcao(0)
                        robot.on_for_distance(SpeedRPM(40),200)
                        j+= 1
                        informacao.posicao+=6
                    
                elif(pathovelha[j].numeroCacifo+1 == pathovelha[j+1].numeroCacifo):#Para o caso de querer andar para a direita 
                    if(pathovelha[j+1].paredeRight == True and pathovelha[j+1].paredeUp == True):
                        sleep(1)
                        sleep(1)
                        coloca_direcao(270)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        j+= 1
                    elif(pathovelha[j+1].paredeRight == True and pathovelha[j+1].paredeDown == True): #tirar o 36 da listaO
                        sleep(1)
                        sleep(1)
                        coloca_direcao(270)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        j+= 1
                    elif ((pathovelha[j+1].paredeRight == True and pathovelha[j+1].numeroCacifo > 30 ) or (pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6] and pathovelha[j+1].paredeUp == True)):
                        sleep(1)
                        coloca_direcao(270)
                        braco.on_for_degrees(100,360)
                        sleep(1)
                        #volta com o braço para cim
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(40),200)
                        coloca_direcao(180)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        informacao.posicao-=6
                        j+= 2
                    elif ((pathovelha[j+1].paredeRight == True and pathovelha[j+1].numeroCacifo < 7) or (pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6] and pathovelha[j+1].paredeDown == True)):
                        sleep(1)
                        
                        coloca_direcao(270)
                        braco.on_for_degrees(100,360)
                        sleep(1)
                        #volta com o braço para cim
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(40),200)
                        coloca_direcao(0)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        informacao.posicao+=6
                        j+= 2
                    else:
                        Sound.beep() 
                        coloca_direcao(270)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        j+= 1
                    
                
                elif(pathovelha[j].numeroCacifo-6 == pathovelha[j+1].numeroCacifo): #para o caso de querer andar para baixo
                    if(pathovelha[j+1].paredeDown == True and pathovelha[j+1].paredeRight == True ):
                        sleep(1)
                        sleep(1)
                        coloca_direcao(180)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao-=6
                        j+= 1
                    elif(pathovelha[j+1].paredeDown == True and pathovelha[j+1].paredeLeft == True ):
                        sleep(1)
                        sleep(1)
                        coloca_direcao(180)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        j+= 1
                    elif((pathovelha[j+1].paredeDown == True and pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6]) or (pathovelha[j+1].numeroCacifo < 7 and pathovelha[j+1].paredeRight == True)):
                        sleep(1)
                        
                        coloca_direcao(180)
                        braco.on_for_degrees(100,360)
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(40),200)
                        coloca_direcao(90)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao-=6
                        informacao.posicao-=1
                        j+= 2
                    elif((pathovelha[j+1].paredeDown == True and pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]) or (pathovelha[j+1].numeroCacifo < 7 and pathovelha[j+1].paredeLeft == True)):
                        sleep(1)
                        
                        coloca_direcao(180)
                        braco.on_for_degrees(100,360)
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(40),200)
                        coloca_direcao(270)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao+=1
                        informacao.posicao-=6
                        j+= 2
                    else:
                        Sound.beep() 
                        coloca_direcao(180)
                        robot.on_for_distance(SpeedRPM(40),200)
                        informacao.posicao-=6
                        j+= 1
                    
                elif(pathovelha[j].numeroCacifo-1 == pathovelha[j+1].numeroCacifo): #para o caso de querer andar para a esquerda 
                    if(pathovelha[j+1].paredeLeft == True and pathovelha[j+1].paredeDown == True ):
                        sleep(1)
                        sleep(1)
                        coloca_direcao(90)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(20),200)
                        informacao.posicao-=1
                        j+= 1
                    elif(pathovelha[j+1].paredeLeft == True and pathovelha[j+1].paredeUp == True):
                        sleep(1)
                        sleep(1)
                        coloca_direcao(90)
                        Sound.beep()
                        robot.on_for_distance(SpeedRPM(20),200)
                        informacao.posicao-=1
                        j+= 1
                    elif((pathovelha[j+1].paredeLeft == True and pathovelha[j+1].numeroCacifo < 6) or (pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1] and pathovelha[j+1].paredeDown == True)):
                        sleep(1)
                    
                        coloca_direcao(90)
                        braco.on_for_degrees(100,360)
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(20),200)
                        coloca_direcao(0)
                        robot.on_for_distance(SpeedRPM(20),200)
                        informacao.posicao-=1
                        informacao.posicao+=6 
                        j+= 2 
                    elif((pathovelha[j+1].paredeLeft == True and pathovelha[j+1].numeroCacifo > 30) or (pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1] and pathovelha[j+1].paredeUp == True)):  
                        sleep(1)
                        coloca_direcao(90)
                        braco.on_for_degrees(100,360)
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        robot.on_for_distance(SpeedRPM(20),200)
                        coloca_direcao(180)
                        robot.on_for_distance(SpeedRPM(20),200)
                        informacao.posicao-=1
                        informacao.posicao-=6 
                        j+= 2
                    else:
                        Sound.beep() 
                        coloca_direcao(90)
                        robot.on_for_distance(SpeedRPM(20),200)
                        informacao.posicao-=1
                        j+= 1
        else:
             #3 paredes
            if(cacifo_ovelha.paredeUp and cacifo_ovelha.paredeDown and (cacifo_ovelha.paredeLeft or cacifo_ovelha.paredeRight)):
                if (cacifo_ovelha.paredeLeft):
                    coloca_direcao(0) #virado para cima
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                    ovelha = ovelha+1
                    coloca_direcao(270) #virado para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                else:
                    coloca_direcao(0) #virado para cima
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                    ovelha = ovelha-1
                    coloca_direcao(90) #virado para a esquerda
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
            elif(cacifo_ovelha.paredeRight and cacifo_ovelha.paredeLeft and (cacifo_ovelha.paredeDown or cacifo_ovelha.paredeUp)):
                if(cacifo_ovelha.paredeDown):
                    coloca_direcao(270) #virado para a direita
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,6) #ovelha vai para cima
                    ovelha = ovelha+6
                    coloca_direcao(0) #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                else:
                    coloca_direcao(270) #virado para a direita
                    Sound.beep() #Apita
                    atualiza_ovelha(ovelha,-6) #ovelha vai para baixo
                    ovelha = ovelha-6
                    coloca_direcao(90) #virado para baixo
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    informacao.posicao-=6 #atualiza a posição do robot a dizer que foi para baixa
            #casos de paredes em L   
            elif(cacifo_ovelha.paredeDown and cacifo_ovelha.paredeRight):
                coloca_direcao(270) #virado para a direita
                Sound.beep() #Apita
                robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                atualiza_ovelha(ovelha,6) #ovelha vai para a cima
                ovelha = ovelha+6
                informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
            elif(cacifo_ovelha.paredeLeft and cacifo_ovelha.paredeUp):
                coloca_direcao(0) #virado para cima
                Sound.beep() #Apita
                robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                ovelha = ovelha+1
                informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
            elif(cacifo_ovelha.paredeUp and cacifo_ovelha.paredeRight):
                coloca_direcao(0) #virado para cima
                Sound.beep() #Apita
                robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                ovelha = ovelha-1
                informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
            elif(cacifo_ovelha.paredeLeft and cacifo_ovelha.paredeDown):
                coloca_direcao(90) #virado para a esquerda
                Sound.beep() #Apita
                robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                atualiza_ovelha(ovelha,6) #ovelha vai para a cima
                ovelha = ovelha+6
                informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
            #casos de 1 parede ou limites
            elif(cacifo_ovelha.paredeUp or cacifo_ovelha.paredeRight or cacifo_ovelha.paredeLeft or cacifo_ovelha.paredeDown or cacifo_ovelha.numeroCacifo in [1,6,31]):
                if(cacifo_ovelha.numeroCacifo in [32,33,34,35]): #caso de estar nos cacifos do topo do tabuleiro
                    if(cacifo_ovelha.paredeRight):
                        coloca_direcao(270) #virado para a direita
                        braco.on_for_degrees(100,360) #mexe o braço para baixo                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360) #mexe o braço para cima
                        atualiza_ovelha(ovelha,-5) #ovelha vai para baixo e para direita
                        ovelha = ovelha-5
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        coloca_direcao(180) #virado para baixo
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao-=6 #atualiza a posição do robot a dizer que foi para baixo
                    else:
                        coloca_direcao(270) #virado para a direita
                        Sound.beep() #Apita
                        atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                        ovelha = ovelha+1
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                elif(cacifo_ovelha.paredeUp):
                    sleep(1)
                    coloca_direcao(270) #virado para direita
                    Sound.beep() #Apita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                    ovelha = ovelha+1
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita    
                elif(cacifo_ovelha.paredeRight):
                    sleep(1)
                    coloca_direcao(0) #virado para cima
                    Sound.beep() #Apita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,6) #ovelha vai para cima
                    ovelha = ovelha+6
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                elif(cacifo_ovelha.paredeDown):
                    sleep(1)
                    coloca_direcao(270) #virado para direita
                    Sound.beep() #Apita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                    ovelha = ovelha+1
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                elif(cacifo_ovelha.paredeLeft):
                    sleep(1)
                    coloca_direcao(0) #virado para cima
                    Sound.beep() #Apita
                    robot.on_for_distance(SpeedRPM(40),200) #anda para direção estabelecida em cima
                    atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                    ovelha = ovelha+1
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                elif(cacifo_ovelha.numeroCacifo in [31]):
                    coloca_direcao(0) #virado para cima
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    atualiza_ovelha(ovelha,2) #ovelha vai 2 para a direita
                    ovelha = ovelha+2
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                    coloca_direcao(270) #virado para a direita
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                elif(cacifo_ovelha.numeroCacifo in [6]):
                    #Usa o braço
                    coloca_direcao(270) #virado para a direita
                    braco.on_for_degrees(100,360) #mexe o braço para baixo                
                    sleep(1)
                    #volta com o braço para cima 
                    braco.on_for_degrees(100,-360) #mexe o braço para cima
                    atualiza_ovelha(ovelha,12) #ovelha vai 2 para cima
                    ovelha = ovelha+12
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                    coloca_direcao(0) #virado para cima
                    robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                    informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                elif(cacifo_ovelha.numeroCacifo in [1]):
                    #Para o caso da ovelha estar no cacifo 1
                    if(informacao.posicao == 2): #robot no cacifo 2
                        coloca_direcao(90)
                        braco.on_for_degrees(100,360)                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        atualiza_ovelha(ovelha,12) #ovelha vai 2 para cima
                        ovelha = ovelha+12
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao-=1 #atualiza a posição do robot a dizer que foi para a esquerda
                        coloca_direcao(0) #virado para cima
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao+=6 #atualiza a posição do robot a dizer que subiu
                    else: #robot no cacifo 7
                        coloca_direcao(180)
                        braco.on_for_degrees(100,360)                
                        sleep(1)
                        #volta com o braço para cima 
                        braco.on_for_degrees(100,-360)
                        atualiza_ovelha(ovelha,2) #ovelha vai 2 para a direita
                        ovelha = ovelha+2
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                        informacao.posicao-=6 #atualiza a posição do robot a dizer que desceu
                        coloca_direcao(270) #virado para a direita
                        informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
                        robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
            

            elif(cacifo_ovelha.numeroCacifo in [2,3,4,5]): #estar nos cacifos da parte de baixo do tabuleiro
                coloca_direcao(270) #virado para a direita
                Sound.beep() #Apita
                atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                ovelha = ovelha+1
                robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                informacao.posicao+=1 #atualiza a posição do robot a dizer que foi para a direita
            elif (cacifo_ovelha.numeroCacifo in [30,24,18,12]): #estar no cacifos mais à direita
                coloca_direcao(0) #virado para cima
                Sound.beep() #Apita
                atualiza_ovelha(ovelha,6) #ovelha vai para cima
                ovelha = ovelha+6
                robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
            elif (cacifo_ovelha.numeroCacifo in [26,27,28,29,20,21,22,23,14,15,16,17,8,9,10,11] and cacifo_ovelha.paredeDown == False and cacifo_ovelha.paredeLeft == False and cacifo_ovelha.paredeUp == False and cacifo_ovelha.paredeRight == False): #Caso de estar num cacifo sem paredes e que não faça parte dos limites do tabuleiro
                coloca_direcao(0) #virado para cima
                Sound.beep() #Apita
                atualiza_ovelha(ovelha,6) #ovelha vai para cima
                ovelha = ovelha+6
                robot.on_for_distance(SpeedRPM(40),200) #mete o robot a andar na direção
                informacao.posicao+=6 #atualiza a posição do robot a dizer que foi para cima
                #ficar por baixo a apitar


        #fim do código novo

            elif(informacao.posicao == cacifo_ovelha.numeroCacifo-6):
                j = 0
                pathovelha = algoritmo_A_star(30,True)
                while(j < len(pathovelha)-1): 
                    if(pathovelha[j].numeroCacifo+6 == pathovelha[j+1].numeroCacifo): #Para o caso de querer subir 
                        if((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeRight == True)): 
                            sleep(1)
                            sleep(1)
                            coloca_direcao(0)
                            Sound.beep()
                            robot.on_for_distance(SpeedRPM(40),200)
                            atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                            ovelha = ovelha-1
                            informacao.posicao+=6
                            j+= 1
                        
                        elif((pathovelha[j+1].paredeUp == True and pathovelha[j+1].paredeLeft == True)):
                            sleep(1)
                            sleep(1)
                            coloca_direcao(0) 
                            Sound.beep()
                            robot.on_for_distance(SpeedRPM(40),200)
                            atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                            ovelha = ovelha+1
                            informacao.posicao+=6
                            j+= 1
                        elif ((pathovelha[j+1].paredeUp == True and pathovelha[j+1].numeroCacifo in [36, 30, 24, 18, 12, 6]) or (pathovelha[j+1].numeroCacifo > 30 and pathovelha[j+1].paredeRight == True)):              
                            if(pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6]):
                                sleep(1)
                                coloca_direcao(0)
                                braco.on_for_degrees(100,360)                
                                sleep(1) 
                                braco.on_for_degrees(100,-360)
                                #volta com o braço para cima
                                atualiza_ovelha(ovelha,5) #ovelha vai para a esquerda e para cima
                                ovelha = ovelha+5
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(90)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=6
                                informacao.posicao-=1
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo >30):
                                sleep(1)
                                coloca_direcao(0)
                                braco.on_for_degrees(100,360)                
                                sleep(1) 
                                braco.on_for_degrees(100,-360)
                                #volta com o braço para cima
                                atualiza_ovelha(ovelha,-5) #ovelha vai para baixo e para a direita
                                ovelha = ovelha-5
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(90)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=6
                                informacao.posicao-=1
                                j+= 2
                        elif ((pathovelha[j+1].paredeUp == True and pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]) or (pathovelha[j+1].numeroCacifo > 30 and pathovelha[j+1].paredeLeft == True)):
                            if(pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]):
                                sleep(1)
                                
                                coloca_direcao(0)
                                braco.on_for_degrees(100,360)                
                                sleep(1) 
                                #volta com o braço para cima
                                atualiza_ovelha(ovelha,6) #ovelha vai para cima e para a direita
                                ovelha = ovelha+6
                                braco.on_for_degrees(100,-360)
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(270)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=6
                                informacao.posicao+=1
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo > 30):
                                sleep(1)
                                
                                coloca_direcao(0)
                                braco.on_for_degrees(100,360)                
                                sleep(1) 
                                #volta com o braço para cima
                                atualiza_ovelha(ovelha,-5) #ovelha vai para baixo e para a direita
                                ovelha = ovelha-5
                                braco.on_for_degrees(100,-360)
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(270)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=6
                                informacao.posicao+=1
                                j+= 2
                        else:
                            Sound.beep() 
                            coloca_direcao(0)
                            atualiza_ovelha(ovelha,6) #ovelha vai para cima
                            ovelha = ovelha+6
                            robot.on_for_distance(SpeedRPM(40),200)
                            j+= 1
                            informacao.posicao+=6
                        
                    elif(pathovelha[j].numeroCacifo+1 == pathovelha[j+1].numeroCacifo):#Para o caso de querer andar para a direita 
                        if(pathovelha[j+1].paredeRight == True and pathovelha[j+1].paredeUp == True):
                            sleep(1)
                            sleep(1)
                            coloca_direcao(270)
                            Sound.beep()
                            atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                            ovelha = ovelha-1
                            robot.on_for_distance(SpeedRPM(40),200)
                            informacao.posicao+=1
                            j+= 1
                        elif(pathovelha[j+1].paredeRight == True and pathovelha[j+1].paredeDown == True): #tirar o 36 da listaO
                            sleep(1)
                            sleep(1)
                            coloca_direcao(270)
                            Sound.beep()
                            atualiza_ovelha(ovelha,6) #ovelha vai para cima
                            ovelha = ovelha+6
                            robot.on_for_distance(SpeedRPM(40),200)
                            informacao.posicao+=1
                            j+= 1
                        elif ((pathovelha[j+1].paredeRight == True and pathovelha[j+1].numeroCacifo > 30 ) or (pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6] and pathovelha[j+1].paredeUp == True)):
                            if(pathovelha[j+1].numeroCacifo > 30):
                                sleep(1)
                                coloca_direcao(270)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,-5) #ovelha vai para baixo e para a direita
                                ovelha = ovelha-5
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(180)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=1
                                informacao.posicao-=6
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6]):
                                sleep(1)
                                coloca_direcao(270)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,5) #ovelha vai para a esquerda e para cima
                                ovelha = ovelha+5
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(180)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=1
                                informacao.posicao-=6
                                j+= 2

                        elif ((pathovelha[j+1].paredeRight == True and pathovelha[j+1].numeroCacifo < 7) or (pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6] and pathovelha[j+1].paredeDown == True)):
                                if(pathovelha[j+1].numeroCacifo < 7):
                                    sleep(1)
                                    coloca_direcao(270)
                                    braco.on_for_degrees(100,360)
                                    sleep(1)
                                    #volta com o braço para cima
                                    braco.on_for_degrees(100,-360)
                                    atualiza_ovelha(ovelha,7) #ovelha vai para cima e para a direita
                                    ovelha = ovelha+7
                                    robot.on_for_distance(SpeedRPM(40),200)
                                    coloca_direcao(0)
                                    robot.on_for_distance(SpeedRPM(40),200)
                                    informacao.posicao+=1
                                    informacao.posicao+=6
                                    j+= 2
                                elif(pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6]):
                                    sleep(1)
                                    
                                    coloca_direcao(270)
                                    braco.on_for_degrees(100,360)
                                    sleep(1)
                                    #volta com o braço para cima
                                    braco.on_for_degrees(100,-360)
                                    atualiza_ovelha(ovelha,5) #ovelha vai para a esquerda e para cima
                                    ovelha = ovelha+5
                                    robot.on_for_distance(SpeedRPM(40),200)
                                    coloca_direcao(0)
                                    robot.on_for_distance(SpeedRPM(40),200)
                                    informacao.posicao+=1
                                    informacao.posicao+=6
                                    j+= 2
                        else:
                            Sound.beep() 
                            coloca_direcao(270)
                            robot.on_for_distance(SpeedRPM(40),200)
                            atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                            ovelha = ovelha+1
                            informacao.posicao+=1
                            j+= 1
                        
                    
                    elif(pathovelha[j].numeroCacifo-6 == pathovelha[j+1].numeroCacifo): #para o caso de querer andar para baixo
                        if(pathovelha[j+1].paredeDown == True and pathovelha[j+1].paredeRight == True ):
                            sleep(1)
                            sleep(1)
                            coloca_direcao(180)
                            Sound.beep()
                            atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                            ovelha = ovelha-1
                            robot.on_for_distance(SpeedRPM(40),200)
                            informacao.posicao-=6
                            j+= 1
                        elif(pathovelha[j+1].paredeDown == True and pathovelha[j+1].paredeLeft == True ):
                            sleep(1)
                            sleep(1)
                            coloca_direcao(180)
                            Sound.beep()
                            atualiza_ovelha(ovelha,1) #ovelha vai para a direita
                            ovelha = ovelha+1
                            robot.on_for_distance(SpeedRPM(40),200)
                            informacao.posicao+=1
                            j+= 1
                        elif((pathovelha[j+1].paredeDown == True and pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6]) or (pathovelha[j+1].numeroCacifo < 7 and pathovelha[j+1].paredeRight == True)):
                            if(pathovelha[j+1].numeroCacifo in [36,30,24,18,12,6]): 
                                sleep(1)
                                
                                coloca_direcao(180)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,-7) #ovelha vai para a esquerda e para baixo
                                ovelha = ovelha-7
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(90)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao-=6
                                informacao.posicao-=1
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo < 7):
                                sleep(1)
                                
                                coloca_direcao(180)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,5) #ovelha vai para a esquerda e para cima
                                ovelha = ovelha+5
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(90)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao-=6
                                informacao.posicao-=1
                                j+= 2
                        elif((pathovelha[j+1].paredeDown == True and pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]) or (pathovelha[j+1].numeroCacifo < 7 and pathovelha[j+1].paredeLeft == True)):
                            if(pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]):
                                sleep(1)
                                
                                coloca_direcao(180)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,-5) #ovelha vai para a direita e para baixo
                                ovelha = ovelha-5
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(270)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=1
                                informacao.posicao-=6
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo < 7):
                                sleep(1)
                                
                                coloca_direcao(180)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,7) #ovelha vai para a direita e para cima
                                ovelha = ovelha+7
                                ovelha = ovelha+7
                                robot.on_for_distance(SpeedRPM(40),200)
                                coloca_direcao(270)
                                robot.on_for_distance(SpeedRPM(40),200)
                                informacao.posicao+=1
                                informacao.posicao-=6
                                j+= 2
                        else:
                            Sound.beep() 
                            coloca_direcao(180)
                            robot.on_for_distance(SpeedRPM(40),200)
                            atualiza_ovelha(ovelha,-6) #ovelha vai para baixo
                            ovelha = ovelha-6
                            informacao.posicao-=6
                            j+= 1
                        
                    elif(pathovelha[j].numeroCacifo-1 == pathovelha[j+1].numeroCacifo): #para o caso de querer andar para a esquerda 
                        if(pathovelha[j+1].paredeLeft == True and pathovelha[j+1].paredeDown == True ):
                            sleep(1)
                            sleep(1)
                            coloca_direcao(90)
                            Sound.beep()
                            robot.on_for_distance(SpeedRPM(20),200)
                            atualiza_ovelha(ovelha,6) #ovelha vai para cima
                            ovelha = ovelha+6
                            informacao.posicao-=1
                            j+= 1
                        elif(pathovelha[j+1].paredeLeft == True and pathovelha[j+1].paredeUp == True):
                            sleep(1)
                            sleep(1)
                            coloca_direcao(90)
                            Sound.beep()
                            robot.on_for_distance(SpeedRPM(20),200)
                            atualiza_ovelha(ovelha,-6) #ovelha vai para baixo
                            ovelha = ovelha-6
                            informacao.posicao-=1
                            j+= 1
                        elif((pathovelha[j+1].paredeLeft == True and pathovelha[j+1].numeroCacifo < 6) or (pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1] and pathovelha[j+1].paredeDown == True)):
                            if(pathovelha[j+1].numeroCacifo < 6):
                                sleep(1)
                            
                                coloca_direcao(90)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,5) #ovelha vai para cima e para a esquerda
                                ovelha = ovelha+5
                                robot.on_for_distance(SpeedRPM(20),200)
                                coloca_direcao(0)
                                robot.on_for_distance(SpeedRPM(20),200)
                                informacao.posicao-=1
                                informacao.posicao+=6 
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]):
                                sleep(1)
                            
                                coloca_direcao(90)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,12) #ovelha vai para cima 2 vezes
                                ovelha = ovelha+12
                                robot.on_for_distance(SpeedRPM(20),200)
                                coloca_direcao(0)
                                robot.on_for_distance(SpeedRPM(20),200)
                                informacao.posicao-=1
                                informacao.posicao+=6 
                                j+= 2

                        elif((pathovelha[j+1].paredeLeft == True and pathovelha[j+1].numeroCacifo > 30) or (pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1] and pathovelha[j+1].paredeUp == True)):  
                            if(pathovelha[j+1].numeroCacifo > 30):
                                sleep(1)
                                coloca_direcao(90)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,-7) #ovelha vai para baixo e para a esquerda
                                ovelha = ovelha-7
                                robot.on_for_distance(SpeedRPM(20),200)
                                coloca_direcao(180)
                                robot.on_for_distance(SpeedRPM(20),200)
                                informacao.posicao-=1
                                informacao.posicao-=6 
                                j+= 2
                            elif(pathovelha[j+1].numeroCacifo in [31,25,19,13,7,1]):
                                sleep(1)
                                coloca_direcao(90)
                                braco.on_for_degrees(100,360)
                                sleep(1)
                                #volta com o braço para cima 
                                braco.on_for_degrees(100,-360)
                                atualiza_ovelha(ovelha,7) #ovelha vai para a direita e para cima
                                ovelha = ovelha+7
                                robot.on_for_distance(SpeedRPM(20),200)
                                coloca_direcao(180)
                                robot.on_for_distance(SpeedRPM(20),200)
                                informacao.posicao-=1
                                informacao.posicao-=6 
                                j+= 2
                        else:
                            Sound.beep() 
                            coloca_direcao(90)
                            robot.on_for_distance(SpeedRPM(20),200)
                            atualiza_ovelha(ovelha,-1) #ovelha vai para a esquerda
                            ovelha = ovelha-1
                            informacao.posicao-=1
                            j+= 1
            else:
                vai_ate_ovelha(ovelha)      


            
def calcula_inicio(ovelha,ignora): #Função que vai calcular em que sitio começar e qual o caminho da heurística que recebe qual a ovelha que quer guiar
    k = CacifoAtual(ovelha)
    cacifo_baixo_direita = CacifoAtual(ovelha-5)
    cacifo_baixo_esquerda = CacifoAtual(ovelha-7)
    cacifo_cima_esquerda = CacifoAtual(ovelha+5)
    cacifo_cima_direita = CacifoAtual(ovelha+7)
    caminho_ovelha = []
    if(k.numeroCacifo in [32,33,34,35]): #se tiver na linha de cima do tabuleiro (o 31 é um caso especial pk tem a parede dos limites)
        if(k.paredeDown):
            if(k.paredeLeft):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
            elif(k.paredeRight):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
                 #se tiver uma parede em L 
             #vai para a posição abaixo da ovelha por causa da parede em L
        elif(k.paredeLeft and k.paredeRight):
            if(cacifo_baixo_esquerda.paredeRight):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha porque fica mais perto da cerca para a ovelha
            else:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif (k.paredeLeft == True) :
            caminho_ovelha =  algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif (k.paredeRight == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeDown == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
    elif (k.numeroCacifo in [31]): #Caso especial para o 31 porque faz parte do limite do tabuleiro de cima e da esquerda (por isso já faz um L)
        if(k.paredeDown == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeRight == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha porque fica mais perto da cerca para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
    elif(k.numeroCacifo in [2,3,4,5]): #se tiver na linha de baixo do tabuleiro (o 6 é um caso especial porque tem a parede em dos limites o 1 não é caso especial porque a ovelha nunca vai estar na posição 1 no inicio)
        if(k.paredeUp):
            if(k.paredeRight):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
            elif(k.paredeLeft:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
             #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeLeft and k.paredeRight):
            if(cacifo_cima_esquerda.paredeRight):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha porque fica mais perto da cerca para a ovelha
            else:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeRight):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif (k.paredeUp ):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeLeft):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha porque fica mais perto da cerca para a ovelha
        else:   
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
    elif(k.numeroCacifo in [6]): #Caso especial para o 6 porque faz parte do limite do tabuleiro em baixo e da direita (por isso já faz um L)
        if(k.paredeUp):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeLeft):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda porque fica mais perto da cerca se bater para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
    elif(k.numeroCacifo in [25,19,13,7]): #se tiver na linha da esquerda do tabuleiro (o 31 não interessa porque já é resolvido num elif de cima)
        if(k.paredeRight):
            #caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
            if(k.paredeDown):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora)
            elif(k.paredeUp):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora)
        elif(k.paredeDown and k.paredeUp): #como se fosse 3 paredes
            if(cacifo_baixo_direita.paredeUp):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
            else:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
        elif(k.paredeUp):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeDown):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
    elif(k.numeroCacifo in [30,24,18,12]): #se tiver na linha da direita do tabuleiro (o 6 não interessa porque já é resolvido num elif de cima)
        if(k.paredeLeft):
            if(k.paredeUp):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
            elif(k.paredeDown):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
             #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeUp and k.paredeDown):
            if(cacifo_baixo_esquerda.paredeUp):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição a cima da ovelha porque fica mais perto da cerca para a ovelha
            else:
               caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha 
        elif(k.paredeLeft or k.paredeUp):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeDown):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
    
    else:#Trata dos casos para L's dentro do tabuleiro
        #Casos de 3 paredes 
        if(k.paredeUp and k.paredeDown and (k.paredeLeft or k.paredeRight)): #Para o caso de ter 3 paredes em forma de C para a direita ou para a esquerda nota: usar lógica negada?
            if(cacifo_baixo_direita.paredeUp):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha    
            else:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeRight and k.paredeLeft and (k.paredeDown or k.paredeUp)): #Para o caso de ter 3 paredes em forma de U para cima ou para baixo
            if(cacifo_cima_esquerda.paredeRight):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha porque fica mais perto da cerca se bater para a ovelha
            else:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        #Casos normais de 2 paredes
        elif(k.paredeUp and k.paredeLeft):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeDown and k.paredeRight):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeLeft and k.paredeDown): 
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
            #opcao 2 caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeUp and k.paredeRight):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        #Casos normais de 1 parede
        elif(k.paredeUp or k.paredeDown): 
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeRight or k.paredeLeft):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
            #  opcao 2 caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1) #vai para a posição à direita da ovelha porque fica mais perto da cerca para a ovelha
    return caminho_ovelha        
        
def escolhe_ovelha(): #função que escolhe qual é a melhor ovelha para começar a ser guiada (para evitar ter de fazer mais ifs na função calcula_inicio() )
    ov1 = CacifoAtual(posicao_ovelhas[0]) #1ª ovelha encontrada
    ov2 = CacifoAtual(posicao_ovelhas[1]) #2º ovelha encontrada
    if (ov1.numeroCacifo == ov2.numeroCacifo+1 or ov1.numeroCacifo == ov2.numeroCacifo+6 or ov1.numeroCacifo == ov2.numeroCacifo-1 or ov1.numeroCacifo == ov2.numeroCacifo-6): #Verifica se a 2ª ovelha está num cacifo adjacente ao da 1ª ovelha   
        if(ov1.numeroCacifo in [35,34,33,32,31,25,19,13,7,2,3,4,5,6,12,18,24,30]): #Verifica se a 1ª ovelha está na borda do tabuleiro
            return posicao_ovelhas[1]  #Devolve a 2ª ovelha porque é a mais fácil para começar a guiar
        
    return posicao_ovelhas[0] #Devolve a 1ª ovelha por motivo nenhum senão simplesmente ser a primeira que foi encontrada
def vai_ate_ovelha(ovelha):
    caminho = calcula_inicio(ovelha,False)
    i=0
    while(i<len(caminho)-1):
        if(caminho[i+1].numeroCacifo-caminho[i].numeroCacifo==6):
            coloca_direcao(0)
        elif(caminho[i+1].numeroCacifo-caminho[i].numeroCacifo==-6):
            coloca_direcao(180)
        elif(caminho[i+1].numeroCacifo-caminho[i].numeroCacifo==1):
            coloca_direcao(270)
        else:
            coloca_direcao(90)
        robot.on_for_distance(SpeedRPM(40), 200)
        atualiza_posicao()
        i+=1


def main():
    inicializaCacifos()
    adiciona_parede(8)
    adiciona_parede(4)
    informacao.direcao=90
    adiciona_parede(33)
    adiciona_parede(18)
    adiciona_parede(26)
    adiciona_parede(22)
    
    informacao.direcao=0
    informacao.posicao=11
    guarda_posicao_ovelha()
    informacao.posicao=26
    guarda_posicao_ovelha()
    informacao.posicao=1

    while ((len(cacifos_visitados)<36)):
        if((paredes_encontradas ==6 and len(posicao_ovelhas)==2)):
            break
        verifica_cacifo()
        #print(array_pode_avancar, file= stderr)
    
    

    if(len(posicao_ovelhas) ==1): #Caso verifique todos os cacifos e só tenha adicionado 1 ovelha (2 ovelhas no mesmo cacifo)
        posicao_ovelhas[1]= posicao_ovelhas[0]

    if(paredes_encontradas==5):
        x = posicao_ovelhas[0]- posicao_ovelhas[1]
        if(x==6):
            informacao.direcao=0
            adiciona_parede(posicao_ovelhas[1])
        elif(x==-6):
            informacao.direcao=180
            adiciona_parede(posicao_ovelhas[1])
        elif(x==1):
            informacao.direcao=270
            adiciona_parede(posicao_ovelhas[1])
        elif(x==-1):
            informacao.direcao=90
            adiciona_parede(posicao_ovelhas[1])
    betty = escolhe_ovelha()

    if (betty == posicao_ovelhas[0]): #escolheu a primeira ovelha
        vitoria = posicao_ovelhas[1]
        vai_ate_ovelha(betty)
        #caminhobetty = algoritmo_A_star(36,True) 
        guia_ovelha(betty)
        #guia_ovelha(caminhobetty)
        vai_ate_ovelha(vitoria)
        guia_ovelha(vitoria)
        #caminhovitoria = algoritmo_A_star(36,True)
        #guia_ovelha(caminhovitoria)

    elif (posicao_ovelhas[0] == posicao_ovelhas[1]):
        vai_ate_ovelha(betty)
        guia_ovelha(betty)
        #caminhobetty = algoritmo_A_star(36,True)
        #guia_ovelha(caminhobetty)

    else:
        vitoria = posicao_ovelhas[0]
        vai_ate_ovelha(betty)
        guia_ovelha(betty)
        vai_ate_ovelha(vitoria)
        guia_ovelha(vitoria)
        

if (__name__ == "__main__"):
    main() 

            



