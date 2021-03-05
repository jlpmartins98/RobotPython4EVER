#!/usr/bin/env python3


# from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank, MediumMotor,SpeedPercent
from math import inf, trunc
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
                child.custoCaminho += 1000
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
    #print("atualiza",file= stderr)
    #print(posicao_ovelhas[0],file= stderr)
    #print(posicao_ovelhas[1],file= stderr)

           

#FUNÇAO QUE CALCULA O CAMINHO DA OVELHA QUANDO o robo USA O BRAÇO
def calcula_braco(posicao_ovelha):
    cacifo_ovelha = CacifoAtual(posicao_ovelha)
    if(informacao.direcao==0): # Virado para cima (Robot por baixo da ovelha)
        if(cacifo_ovelha.numeroCacifo in [32,33,34,35]): # Se a ovelha estiver no limite superior do tabuleiro 
            if(cacifo_ovelha.paredeRight == True): # Se tiver parede ao lado direito da ovelha (ovelha vai para a esquerda) -1 ||
                proximo_cacifo = CacifoAtual(posicao_ovelha-1) 
                if(proximo_cacifo.paredeLeft == True or proximo_cacifo.numeroCacifo in [1,7,13,19,25,31]): # Depois do primeiro movimento tem parede à esquerda (ovelha desce) -6
                    return posicao_ovelha-7
                else: # Não tem parede à esquerda, ovelha anda 2 casas para esquerda
                    return posicao_ovelha-2
            else: # Não tem parede do lado direito (ovelha anda para a direita) +1
                proximo_cacifo = CacifoAtual(posicao_ovelha+1)
                if(proximo_cacifo.paredeRight == True): # Depois do primeiro movimento tem parede à direita (ovelha desce) -6
                    return posicao_ovelha-5
                else: # Não tem parede à direita, ovelha anda 2 casas para a direita
                    return posicao_ovelha+2 
        elif(cacifo_ovelha.numeroCacifo == 31): # Vai para a direita, supostamente nunca chamará com parede a impedir o movimento para a direita +1
            proximo_cacifo = CacifoAtual(posicao_ovelha+1)
            if(proximo_cacifo.paredeRight == True): # Depois do primeiro movimento tem parede à direita (ovelha desce) -6
                return posicao_ovelha-5
            else: # Não tem parede à direita, ovelha anda 2 casas para a direita
                return posicao_ovelha+2 
        elif(cacifo_ovelha.paredeUp == True): # Se tiver parede por cima da ovelha e a ovelha não está no limite superior
            if(cacifo_ovelha.paredeRight==True or cacifo_ovelha.numeroCacifo in [6,12,18,24,30]): # Se tiver parede em cima e à direita ou na coluna da direita ( anda para a esquerda) -1
                proximo_cacifo = CacifoAtual(posicao_ovelha-1)
                if(proximo_cacifo.paredeUp == True):# Parede a impedir subir
                    if(proximo_cacifo.paredeLeft == True): # Parede a impedir subir e ir para a esquerda (ovelha desce) -6
                        return posicao_ovelha-7 # Andou para esquerda e baixo
                    else:
                        return posicao_ovelha-2 # Andou para esquerda 2 vezes
                else: # Não tem parede a cima, ela sobe +6
                    return posicao_ovelha+5 # Andou para esquerda e cima
            else: # Não tem parede à direita, anda para a direita +1
                proximo_cacifo = CacifoAtual(posicao_ovelha+1)
                if(proximo_cacifo.paredeUp == True):# Parede a impedir subir
                    if(proximo_cacifo.paredeRight == True): # Parede a impedir subir e ir para a direita (ovelha desce) -6
                        return posicao_ovelha-5 # Andou para direita e baixo
                    else:
                        return posicao_ovelha+2 # Andou para direita 2 vezes
                else: # não tem parede à cima, ela sobe +6
                    return posicao_ovelha+7 # Andou para a direita e subiu
        else: # não há parede a cima e não está no limite superior (ovelha sobe) +6
            proximo_cacifo= CacifoAtual(posicao_ovelha+6)
            if(proximo_cacifo.paredeUp == True or proximo_cacifo.numeroCacifo in [31,32,33,34,35,36]):
                if(proximo_cacifo.paredeRight == True or proximo_cacifo.numeroCacifo in [6,12,18,24,30]): # Há parede na direita ou esta na coluna da direita (ovelha anda para a esquerda)
                    return posicao_ovelha+5 # ovelha subiu e andou para a esquerda
                else: # Não há parede nem está no limite (ovelha para a direita) +1
                    return posicao_ovelha+7 # ovelha subiu e andou para a direita
            else: # Não há parede a impedir a subida
                return posicao_ovelha+12 #ovelha sobe 2 vezes
    elif(informacao.direcao == 90): # Virado para a esquerda
        if(cacifo_ovelha.numeroCacifo in [7,13,19,25]):
            if(cacifo_ovelha.paredeUp == True): # Desce -6
                proximo_cacifo = CacifoAtual(posicao_ovelha-6)
                if(proximo_cacifo.paredeDown == True): # direita +1
                    return posicao_ovelha -5
                else:
                    return posicao_ovelha -12
            else: # Sobe +6
                proximo_cacifo = CacifoAtual(posicao_ovelha+6)
                if(proximo_cacifo.paredeUp == True or proximo_cacifo.numeroCacifo ==31): # direita +1
                    return posicao_ovelha +7
                else:
                    return posicao_ovelha +12
        elif(cacifo_ovelha.numeroCacifo ==1):
            proximo_cacifo = CacifoAtual(posicao_ovelha+6) # sobe +6
            if(proximo_cacifo.paredeUp == True): # direita +1
                return posicao_ovelha +7
            else: # sobe +6
                return posicao_ovelha +12
        elif(cacifo_ovelha.numeroCacifo==31):
            proximo_cacifo = CacifoAtual(posicao_ovelha-6) # desce -6
            if(proximo_cacifo.paredeDown == True): # direita +1
                return posicao_ovelha -5
            else: # desce -6
                return posicao_ovelha -12
        elif(cacifo_ovelha.paredeLeft == True):
            if(cacifo_ovelha.paredeUp == True or cacifo_ovelha.numeroCacifo in [31,32,33,34,35,36]): # baixo -6
                proximo_cacifo = CacifoAtual(posicao_ovelha-6)
                if(proximo_cacifo.paredeLeft == True): # desce ou direita
                    if(proximo_cacifo.paredeDown == True or proximo_cacifo.numeroCacifo in [1,2,3,4,5,6]): # direita +1
                        return posicao_ovelha -5
                    else:# desce -6
                        return posicao_ovelha -12
                else: # esquerda -1
                    return posicao_ovelha -7
            else: # sobe +6
                proximo_cacifo = CacifoAtual(posicao_ovelha +6)
                if(proximo_cacifo.paredeLeft == True): # sobe ou direita
                    if(proximo_cacifo.paredeUp == True or proximo_cacifo.numeroCacifo in [31,32,33,34,35,36]):
                        return posicao_ovelha +7 # direita +1
                    else: # sobe +6
                        return posicao_ovelha+12
                else:# esquerda -1
                    return posicao_ovelha+5
        else: # andou para a esquerda -1
            proximo_cacifo = CacifoAtual(posicao_ovelha-1)
            if(proximo_cacifo.paredeLeft == True or proximo_cacifo.numeroCacifo in [1,7,13,19,25,31]):
                if(proximo_cacifo.paredeUp==True or proximo_cacifo.numeroCacifo == 31):
                    return posicao_ovelha-7 # desce -6
                else:
                    return posicao_ovelha +5 # sobe +6
            else:
                return posicao_ovelha -2 # esquerda -1
    
    elif(informacao.direcao == 180): # Virado para baixo
        if(cacifo_ovelha.numeroCacifo in [2,3,4,5]):
            if(cacifo_ovelha.paredeLeft == True): # direta +1
                proximo_cacifo = CacifoAtual(posicao_ovelha+1)
                if(proximo_cacifo.paredeRight == True or proximo_cacifo ==6): # Sobe +6
                    return posicao_ovelha +7
                else: # direita +1
                    return posicao_ovelha +2
            else: # esquerda -1
                proximo_cacifo = CacifoAtual(posicao_ovelha-1)
                if(proximo_cacifo.paredeLeft == True or proximo_cacifo.numeroCacifo ==1): # sobe +1
                    return posicao_ovelha +5
                else:
                    return posicao_ovelha -2
        elif(cacifo_ovelha.numeroCacifo ==1):
            proximo_cacifo = CacifoAtual(posicao_ovelha+1) # direita +1
            if(proximo_cacifo.paredeRight == True): # sobe +6
                return posicao_ovelha +7
            else: # direita +1
                return posicao_ovelha +2
        elif(cacifo_ovelha.numeroCacifo==6):
            proximo_cacifo = CacifoAtual(posicao_ovelha-1) # esquerda -1
            if(proximo_cacifo.paredeLeft == True): # sobe +6
                return posicao_ovelha +5
            else: # esquerda -1
                return posicao_ovelha -2
        elif(cacifo_ovelha.paredeDown == True):
            if(cacifo_ovelha.paredeLeft == True or cacifo_ovelha.numeroCacifo in [1,7,13,19,25,31]): # direita +1
                proximo_cacifo = CacifoAtual(posicao_ovelha+1)
                if(proximo_cacifo.paredeDown == True): # sobe ou esquerda
                    if(proximo_cacifo.paredeRight == True or proximo_cacifo.numeroCacifo in [6,12,18,24,30,36]): # sobe +6
                        return posicao_ovelha +7 # sobe +6
                    else:# direita -1
                        return posicao_ovelha +2
                else: # baixo -6
                    return posicao_ovelha -5
            else: # esquerda -1
                proximo_cacifo = CacifoAtual(posicao_ovelha -1)
                if(proximo_cacifo.paredeDown == True): # sobe ou esquerda
                    if(proximo_cacifo.paredeLeft == True or proximo_cacifo.numeroCacifo in [1,7,13,19,25,31]):
                        return posicao_ovelha +5 # sobe +6
                    else: # esquerda -1
                        return posicao_ovelha-2
                else:# desce -6
                    return posicao_ovelha-7
        else: # andou para a baixo -6
            proximo_cacifo = CacifoAtual(posicao_ovelha-6)
            if(proximo_cacifo.paredeDown == True or proximo_cacifo.numeroCacifo in [1,2,3,4,5,6]):
                if(proximo_cacifo.paredeLeft==True or proximo_cacifo.numeroCacifo == 1):
                    return posicao_ovelha-5 # direita +1
                else:
                    return posicao_ovelha -7 # esquerda -1
            else:
                return posicao_ovelha -12 # baixo -6
    elif(informacao.direcao == 270): # Virado para a direita
        if(cacifo_ovelha.numeroCacifo in [12,18,24,30]):
            if(cacifo_ovelha.paredeDown == True): # Sobe +6
                proximo_cacifo = CacifoAtual(posicao_ovelha+6)
                if(proximo_cacifo.paredeUp == True): # Esquerda -1
                    return posicao_ovelha +5
                else:
                    return posicao_ovelha +12
            else: # desce -6
                proximo_cacifo = CacifoAtual(posicao_ovelha-6)
                if(proximo_cacifo.paredeDown == True or proximo_cacifo.numeroCacifo ==6): # esquerda -1
                    return posicao_ovelha -7
                else:
                    return posicao_ovelha -12
        elif(cacifo_ovelha.numeroCacifo ==6):
            proximo_cacifo = CacifoAtual(posicao_ovelha+6) # sobe +6
            if(proximo_cacifo.paredeUp == True): # esquerda -1
                return posicao_ovelha +5
            else: # sobe +6
                return posicao_ovelha +12
        elif(cacifo_ovelha.paredeRight == True):
            if(cacifo_ovelha.paredeDown == True or cacifo_ovelha.numeroCacifo in [1,2,3,4,5,6]): # sobe +6
                proximo_cacifo = CacifoAtual(posicao_ovelha+6)
                if(proximo_cacifo.paredeRight == True): # Sobe ou esquerda
                    if(proximo_cacifo.paredeUp == True or proximo_cacifo.numeroCacifo in [31,32,33,34,35]): # direita +1
                        return posicao_ovelha +5 # esquerda -1
                    else:# sobe +6
                        return posicao_ovelha +12
                else: # direita +1
                    return posicao_ovelha +7
            else: # desce -6
                proximo_cacifo = CacifoAtual(posicao_ovelha -6)
                if(proximo_cacifo.paredeRight == True): # desce ou esquerda
                    if(proximo_cacifo.paredeDown == True or proximo_cacifo.numeroCacifo in [1,2,3,4,5,6]):
                        return posicao_ovelha -7 # esquerda -1
                    else: # desce -6
                        return posicao_ovelha-12
                else:# direita +1
                    return posicao_ovelha-5
        else: # andou para a direta +1
            proximo_cacifo = CacifoAtual(posicao_ovelha+1)
            if(proximo_cacifo.paredeRight == True or proximo_cacifo.numeroCacifo in [6,12,18,24,30,36]):
                if(proximo_cacifo.paredeDown==True or proximo_cacifo.numeroCacifo == 6):
                    return posicao_ovelha+7 # sobe +6
                else:
                    return posicao_ovelha -5 # desce -6
            else:
                return posicao_ovelha +2 # direita -1



#funçao que interage com as ovelhas pela 1º vez e devolve a posiçao atualizada da ovelha
#podemos usar esta funçao num ciclo ate a posiçao da ovelha ser 36, esta funçao juntamente com o calcula inicio nas posiçoes atualizadas+ a funçao q leva o robo para a posiçao do calcula_inicio
def interage_ovelha(ovelha):
    #variavel para verificar se a ovelha tem paredes a volta
    if(verifica_adjacentes(posicao_ovelhas[0],posicao_ovelhas[1]) == True):
        return trata_ovelhas_adjacentes_robot(ovelha)
    cacifoRobot = CacifoAtual(informacao.posicao)
    cacifoOvelha = CacifoAtual(ovelha)
    posicao_atualizada_ovelha = ovelha
    #casos cantos
    if(ovelha == 31):#caso a ovelha esteja no cacifo 31
        if(cacifoOvelha.paredeDown):
            #caso tenha uma parede em baixo, o robo vai para baixo da ovelha
            #robo bate, ovelha anda para a direita, robo segue
            #print("15",file=stderr)
            Sound.beep()
            #move_robot(270)
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeRight):
            #caso tenha uma parede a direita, o robo vai para a direita da ovelha
            #robo bate, ovelha anda para baixo 2x, robo segue
            coloca_direcao(90)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        else:#caso sem paredes
            #robo apita, ovelha anda para a direita, robo segue
            #print("16",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
    elif(ovelha == 6):
        if(cacifoOvelha.paredeUp):
            #caso tenha uma parede em cima, robo vai para cima da ovelha
            #robo bate, ovelha anada para a esquerda , robo segue
            #print("17",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeLeft):
            #caso tenha uma parede a esquerda, robo vai para a esquerda da ovelha
            #robo bate, ovelha anda para cima 2x, robo segue
            coloca_direcao(270)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        else: #caso sem paredes
            #robo vai para a esquerda da ovelha
            #robo apita, ovelha sobe, robo segue
            #print("18",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
    elif(ovelha == 1):#canto inferior esquerdo
        if(cacifoOvelha.paredeUp):
            #caso tenha uma parede em cima, robo vai para cima da ovelha
            #robo bate, ovelha anda para a direita 2x, robo segue
            coloca_direcao(180)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        #elif(cacifoOvelha.paredeRight):
            #caso tenha uma parede a direita, robo vai para a direita da ovelha
            #robo apita, ovelha anda para cima, robo segue
            #Sound.beep()
            #return calcula_apito(ovelha)
        else: #caso sem paredes
            #robo vai para a direita da ovelha
            #robo apita, ovelha sobe, robo segue
            #print("19",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)

    #CASOS QUE A OVELHA ESTA NO LIMITE MAS FAZ UM L
    #7,2,5,12,25,32
    
    #CASOS LIMITES
    elif(ovelha in [2,3,4,5]):#limnite inferior
        if(cacifoOvelha.paredeUp and cacifoOvelha.paredeLeft):
            #robo vai para cima da ovelha
            #robo apita, ovelha anda para a direita, robo segue
            #print("20",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeUp and cacifoOvelha.paredeRight):
            #robo vai para cima
            #robo apita, ovelha anda para a esquerda, robo segue
            #print("21",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeLeft or cacifoOvelha.paredeRight):
            #robo vai para a direita ou esquerda (por esta ordem)
            #robo apita, ovelha sobe para cima, robo segue
            #print("23",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        else:#caso nao tenha paredes e esteja no limite inferior
            #robo vai para a esquerda da ovelha
            #robo apita, ovelha anda para a direita, robo segue
            #print("24",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        
    elif(ovelha in {12,18,24,30}):#limite direito
        if(cacifoOvelha.paredeUp and cacifoOvelha.paredeLeft):
            #robo vai para a esquerda da ovelha
            #robo apita, ovelha desce, robo segue
            #print("25",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeDown and cacifoOvelha.paredeLeft):
            #robo vai para a esquerda da ovelha
            #robo bate, ovelha sobe 2x, robo segue
            coloca_direcao(270)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        elif(cacifoOvelha.paredeDown):
            #robo vai para a esquerda da ovelha
            #robo apita, ovelha sobe, robo segue
            #print("26",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeUp):
            #robo vai para baixo da ovelha
            #robo apita, ovelha vai para a esquerda, robo segue
            #print("27",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)
        else:#sem paredes a volta da ovelha ou so uma parede a esquerda
            #robo vai para baixo da ovelha
            #robo apita, ovelha sobe, robo segue
            #print("28",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)

    elif(ovelha in [7,13,19,25]):#limite esquerdo
        if(cacifoOvelha.paredeRight and cacifoOvelha.paredeDown):
            #robo vai para a direita
            #robo usa braço, ovelha sobe 2x, robo segue
            coloca_direcao(90)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        elif(cacifoOvelha.paredeRight and cacifoOvelha.paredeUp):
            #robo vai para a direita
            #robo usa braço, ovelha desce 2x, robo segue
            coloca_direcao(90)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        #elif(cacifoOvelha.paredeUp or cacifoOvelha.paredeDown):
            #robo vai para baixo     robo vai para cima
            #robo apita, ovelha anda para a direita, robo segue
            #print("TESTE 13",file=stderr)
            #Sound.beep()
            #return calcula_apito(ovelha)
        else: #casos sem paredes ou so uma parede a direita
            #robo vai para baixo
            #robo apita, ovelha sobe, robo segue
            #print("1",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)

    elif(ovelha in [32,33,34,35]):#limite superior
        if(cacifoOvelha.paredeDown and cacifoOvelha.paredeRight):
            #robo vai para baixo
            #robo bate, ovelha anda para a esquerda 2x, robo segue
            coloca_direcao(0)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        elif(cacifoOvelha.paredeDown and cacifoOvelha.paredeLeft):
            #robo vai para baixo
            #robo bate, ovelha anda para a direita 2x, robo segue
            coloca_direcao(0)
            braco.on_for_degrees(100,360) #mexe o braço para baixo
            sleep(1)
            braco.on_for_degrees(100,-360) #mexe o braço para cima
            return calcula_braco(posicao_atualizada_ovelha)
        elif(cacifoOvelha.paredeRight):
            #robo vai para a esquerda
            #robo apita, ovelha anda para baixo, robo segue
            #print("2",file=stderr)
            coloca_direcao(270)
            Sound.beep()
            return calcula_apito(ovelha)
        else:#caso sem paredes, apenas 1 parede a esquerda, ou apenas 1 parede em baixo
            #robo vai para a esquerda   #robo vai para baixo    #robo vai para a esquerda
            #robo apita, ovelha anda para a direita, robo segue
            if(cacifoRobot.numeroCacifo +6 == ovelha):
                coloca_direcao(0)
            elif(cacifoRobot.numeroCacifo+1 == ovelha):
                coloca_direcao(270)
            #print("3",file=stderr)
            Sound.beep()
            return calcula_apito(ovelha)

    #casos 3 paredes com a de cima e baixo fixas
    elif(cacifoOvelha.paredeUp and cacifoOvelha.paredeDown and (cacifoOvelha.paredeRight or cacifoOvelha.paredeLeft)):
        #neste caso o robo iria estar em baixo da ovelha
        if(cacifoOvelha.paredeLeft):#caso a parede livre seja a direita
            #caso o robo n tenha uma parede a direita
            #o robo iria apitar e a ovelha ia andar para a direita e o robo vai seguir
            #como o beep é a volta do robo n intressa a direçao para onde esta virado
            coloca_direcao(0)
            #print("4",file=stderr)
            Sound.beep()
            #move_robot(270)
            coloca_direcao(0)
            return calcula_apito(ovelha)
        #caso o robo n consiga andar para a direita ele perde
        else:#caso a parede livre seja a esquerda
            #caso o robo nao tenha uma parede a esquerda
            #o robo iria apitar e a ovelha ia andar para a esquerda e o robo vai seguir
            #como o beep é a volta do robo n intressa a direçao para onde esta virado
            coloca_direcao(0)
            #print("5",file=stderr)
            Sound.beep()
            #move_robot(90)
            coloca_direcao(0)
            return calcula_apito(ovelha)
        #caso o robo n consiga andar para a esquerda ele perde

    #casos 3 paredes com as dos lados fixas
    elif(cacifoOvelha.paredeLeft and cacifoOvelha.paredeRight and (cacifoOvelha.paredeDown or cacifoOvelha.paredeUp)):
        #neste caso o robo iria estar a esquerda da ovelha
        if(cacifoOvelha.paredeDown):#caso a parede livre seja em cima
            #o robo vai apitar, a ovelha vai andar para cima 
            #o robo ira andar para cima
            coloca_direcao(270) 
            #print("6",file=stderr)
            Sound.beep()
            #move_robot(0)
            return calcula_apito(ovelha)
        elif(cacifoOvelha.paredeUp):#caso a parede livre esteja em baixo
            #o robo vai apitar, a ovelha vai andar para baixo 
            #o robo ira andar para baixo 
            coloca_direcao(270)
            #print("7",file=stderr)
            Sound.beep()
            #move_robot(180)
            return calcula_apito(ovelha)

    #casos L
    if(cacifoOvelha.paredeLeft and cacifoOvelha.paredeDown):
        #o robo vai para a direita da ovelha
        #o robo vai apitar, a ovelha vai subir e o robo vai seguir
        #o robo anda para cima
        coloca_direcao(90)
        #print("8",file=stderr)
        Sound.beep()
        #move_robot(0)
        return calcula_apito(ovelha)
    elif(cacifoOvelha.paredeLeft and cacifoOvelha.paredeUp):
        #o robo vai para baixo da ovelha
        #o robo vai apitar, a ovelha vai para a direita e o robo vai seguir
        coloca_direcao(0)
        #print("9",file=stderr)
        Sound.beep()
        #move_robot(270)
        return calcula_apito(ovelha)
    elif(cacifoOvelha.paredeRight and cacifoOvelha.paredeDown):
        #o robo vai para a esquerda da ovelha
        #o robo vai apitar, a ovelha vai para cima e o robo ira seguir
        coloca_direcao(270)
        #print("10",file=stderr)
        Sound.beep()
        #move_robot(0)
        return calcula_apito(ovelha)
    elif(cacifoOvelha.paredeRight and cacifoOvelha.paredeUp):
        #o robo vai para baixo
        #o robo apita, ovelha vai para a esquerda
        coloca_direcao(0)
        print("11",file=stderr)
        Sound.beep()
        #move_robot(90)
        return calcula_apito(ovelha)
    
    #casos 2 paredes paralelas e #casos 1 parede, sao iguais
    elif(cacifoOvelha.paredeUp or cacifoOvelha.paredeDown):
        #vai para a esquerda da ovelha
        #robo apita ovelha anda para a direita, robo segue
        coloca_direcao(270)
        #print("12",file=stderr)
        Sound.beep()
        #move_robot(270)
        return calcula_apito(ovelha)
    elif(cacifoOvelha.paredeRight or cacifoOvelha.paredeLeft):
        #vai para baixo da ovelha
        #robo apita ovelha anda para cima, robo segue
        coloca_direcao(0)
        #print("13",file=stderr)
        Sound.beep()
        #move_robot(0)
        return calcula_apito(ovelha)
    else:#caso sem paredes
        #robo vai para baixo da ovelha
        #robo apita ovelha sobe, robo segue
        coloca_direcao(0)
        #print("14",file=stderr)
        Sound.beep()
        return calcula_apito(ovelha)

def calcula_apito(posicao_ovelha):
    print("Posicao Robo", file=stderr)
    print(informacao.posicao,file=stderr)
    print("Ovelha",file=stderr)
    print(posicao_ovelha, file=stderr)
    direcao = informacao.posicao - posicao_ovelha # robot- ovelha
    #print(direcao)
    cacifo_ovelha = CacifoAtual(posicao_ovelha)
    if(direcao == -6 or direcao == -12): # robot por baixo da ovelha
        if(cacifo_ovelha.numeroCacifo in [31,32,33,34,35,36] or cacifo_ovelha.paredeUp == True):
            if(cacifo_ovelha.paredeRight == True or cacifo_ovelha.numeroCacifo in [6,12,18,24,30,36]):
                atualiza_ovelha(posicao_ovelha,-1)
                return posicao_ovelha -1
            else:
                atualiza_ovelha(posicao_ovelha,1)
                return posicao_ovelha +1
        else:
            atualiza_ovelha(posicao_ovelha,6)
            return posicao_ovelha +6        
    elif(direcao == 6 or direcao ==12): # robot por cima
        if(cacifo_ovelha.numeroCacifo in [1,2,3,4,5,6] or cacifo_ovelha.paredeDown == True):
            if(cacifo_ovelha.paredeLeft == True or cacifo_ovelha.numeroCacifo in [1,7,13,19,25,31]):
                atualiza_ovelha(posicao_ovelha,1)
                return posicao_ovelha +1
            else:
                atualiza_ovelha(posicao_ovelha,-1)
                return posicao_ovelha -1
        else:
            atualiza_ovelha(posicao_ovelha,-6)
            return posicao_ovelha -6
    elif(direcao == 1 or direcao==2): # robot direita
        if(cacifo_ovelha.numeroCacifo in [1,7,13,19,25,31] or cacifo_ovelha.paredeLeft == True):
            if(cacifo_ovelha.paredeUp == True or cacifo_ovelha.numeroCacifo in [31,32,33,34,35,36]):
                atualiza_ovelha(posicao_ovelha,-6)
                return posicao_ovelha -6
            else:
                atualiza_ovelha(posicao_ovelha,6)
                return posicao_ovelha +6
        else:
            atualiza_ovelha(posicao_ovelha,-1)
            return posicao_ovelha -1
    elif(direcao == -1 or direcao == -2): # robot esquerda
        if(cacifo_ovelha.numeroCacifo in [6,12,18,24,30,36] or cacifo_ovelha.paredeRight == True):
            if(cacifo_ovelha.paredeDown == True or cacifo_ovelha.numeroCacifo in [1,2,3,4,5,6]):
                atualiza_ovelha(posicao_ovelha,6)
                return posicao_ovelha +6
            else:
                atualiza_ovelha(posicao_ovelha,-6)
                return posicao_ovelha -6
        else:
            atualiza_ovelha(posicao_ovelha,1)
            return posicao_ovelha +1


#funçao que interage com as ovelhas caso estejam ambas adjacentes ao robot (usa o braço nao o apito)
def trata_ovelhas_adjacentes_robot(ovelha):
    #variavel para saber se o robo esta em cima,baixo, esquerda ou direita da ovelha
    direcao = informacao.posicao - ovelha
    if(direcao == -6):#robo esta em baixo da ovelha
        coloca_direcao(0)
        braco.on_for_degrees(100,360) #mexe o braço para baixo
        sleep(1)
        braco.on_for_degrees(100,-360) #mexe o braço para cima
        return calcula_braco(ovelha)
    elif(direcao == 6):#robo em cima da ovelha
        coloca_direcao(180)
        braco.on_for_degrees(100,360) #mexe o braço para baixo
        sleep(1)
        braco.on_for_degrees(100,-360) #mexe o braço para cima
        return calcula_braco(ovelha)
    elif(direcao == 1):#robo a direita da ovelha
        coloca_direcao(90)
        braco.on_for_degrees(100,360) #mexe o braço para baixo
        sleep(1)
        braco.on_for_degrees(100,-360) #mexe o braço para cima
        return calcula_braco(ovelha)
    elif(direcao == -1):#robo a esquerda da ovelha
        coloca_direcao(270)
        braco.on_for_degrees(100,360) #mexe o braço para baixo
        sleep(1)
        braco.on_for_degrees(100,-360) #mexe o braço para cima
        return calcula_braco(ovelha)



#funcao que verifica se o robo tem as duas ovelhas adjacentes a ele
#recebe a posiçao da ovelha que esta guiando e a posiçao da outra ovelha
#devolve true caso ambas as ovelhas estejam adjacentes ao robo
def verifica_adjacentes(ovelhaGuiando,ovelha2):
    posicao_robot = informacao.posicao
    esquerda_robot = posicao_robot - 1
    direita_robot = posicao_robot + 1
    baixo_robot = posicao_robot - 6
    cima_robot = posicao_robot + 6
    if(cima_robot == ovelhaGuiando):
        if(esquerda_robot == ovelha2):
            return True
        if(direita_robot == ovelha2):
            return True
        if(baixo_robot == ovelha2):
            return True
    if(baixo_robot == ovelhaGuiando):
        if(cima_robot == ovelha2):
            return True
        if(direita_robot == ovelha2):
            return True
        if(esquerda_robot == ovelha2):
            return True
    if(esquerda_robot == ovelhaGuiando):
        if(cima_robot == ovelha2):
            return True
        if(direita_robot == ovelha2):
            return True
        if(baixo_robot == ovelha2):
            return True
    if(direita_robot == ovelhaGuiando):
        if(cima_robot == ovelha2):
            return True
        if(esquerda_robot == ovelha2):
            return True
        if(baixo_robot == ovelha2):
            return True
    return False

                       
def calcula_inicio(sheep,ignora,HerdingSheep): #Função que vai calcular em que sitio começar e qual o caminho da heurística que recebe qual a ovelha que quer guiar
    
    ignora = False
    k = CacifoAtual(sheep)
    caminho_ovelha = []
    #posicoes da ovelha 
    esquerdaOvelhaAux = k.numeroCacifo - 1
    esquerdaOvelha = CacifoAtual(esquerdaOvelhaAux) 
    #direitaOvelha = CacifoAtual(ovelha+1)
    baixoOvelhaAux = k.numeroCacifo - 6 
    baixoOvelha = CacifoAtual(baixoOvelhaAux)
    #cimaOvelha = CacifoAtual(ovelha+6) 
    #mudar o caminho do robot manualmente caso o caminho_ovelha seja maior que 2
    #if(len(caminho_ovelha)>=2 and guiando_a_ovelha):
        #ver cacifo da ovelha
        #12,2,5,7

    if(k.numeroCacifo in [32,33,34,35]): #se tiver na linha de cima do tabuleiro (o 31 é um caso especial pk tem a parede dos limites)
        if(k.paredeDown == True and (k.paredeRight == True or k.paredeLeft == True)): #se tiver uma parede em L 
            #nao posso tocar se nao perco a ovelha
            #restantes ja cobre todos os casos
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha por causa da parede em L
        elif (k.paredeLeft == True):
            caminho_ovelha =  algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif (k.paredeRight == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeDown == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
    #caso a ovelha vai para o cacifo 1
    elif(k.numeroCacifo == 1):
        #perde
        #if(k.paredeUp and k.numeroCacifo+6.paredeRight):
            #caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
          #o robo terá que ir para o cacifo de cima da ovelha
        if(k.paredeUp):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
        elif(k.paredeRight):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora)
        #caso sem paredes
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
 
    elif (k.numeroCacifo in [31]): #Caso especial para o 31 porque faz parte do limite do tabuleiro de cima e da esquerda (por isso já faz um L)
        if(k.paredeDown == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeRight == True):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha porque fica mais perto da cerca para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
    elif(k.numeroCacifo in [2,3,4,5]): #se tiver na linha de baixo do tabuleiro (o 6 é um caso especial porque tem a parede em dos limites o 1 não é caso especial porque a ovelha nunca vai estar na posição 1 no inicio)
        if(k.paredeUp and (k.paredeLeft or k.paredeRight)):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        elif (k.paredeUp or k.paredeRight):
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
        elif(k.paredeUp):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeDown):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
    elif(k.numeroCacifo in [30,24,18,12]): #se tiver na linha da direita do tabuleiro (o 6 não interessa porque já é resolvido num elif de cima)
        if(k.paredeLeft and (k.paredeUp or k.paredeDown)):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeLeft or k.paredeUp):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        elif(k.paredeDown):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)
    
    elif(sheep in [8,9,10,11,14,15,16,17,20,21,22,23,26,27,28,29]):#Trata dos casos para L's dentro do tabuleiro
        #Casos de 3 paredes 
        if(k.paredeUp and k.paredeDown and (k.paredeLeft or k.paredeRight)): #Para o caso de ter 3 paredes em forma de C para a direita ou para a esquerda nota: usar lógica negada?
                #caso tenha parede à esquerda/direita do cacifo debaixo da ovelha, mandar para cima da ovelha
                if(baixoOvelha.paredeLeft or baixoOvelha.paredeRight):
                    caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
                #caso nao tenha parede no cacifo abaixo, o robot poderá ir 
                else:
                    caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)

        elif(k.paredeRight and k.paredeLeft): #Para o caso de ter 3 paredes em forma de U para cima ou para baixo
            #se tiver U virado para baixo, e parede em baixo do cacifo da esquerda é melhor ir para a direita
            if(k.paredeUp):
                if(esquerdaOvelha.paredeDown):
                    caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora)
                #se tiver em cima pode ficar na esquerda
                else:
                    caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)
            #se tiver U e parede em cima no cacifo da esquerda é melhor o robot ir para o cacifo da direita da ovelha
            elif(k.paredeDown):
                if(esquerdaOvelha.paredeUp):
                    caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora)
                else:
                    #se tive em baixo pode ir para a esquerda 
                    caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora)

        #Casos normais de 2 paredes
        elif(k.paredeUp and k.paredeLeft):
            #caso tenha uma parede na direita no cacifo abaixo, mandar para o cacifo de cima da ovelha
            if(baixoOvelha.paredeRight):
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo+6,ignora)
            #pode ficar em baixo pois nao perde a ovelha
            else:
                caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)

        #neste caso o robo entra vai para o cacifo que a ovelha estava por isso nao há outra
        elif(k.paredeDown and k.paredeRight):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha 
        #neste tbm
        elif(k.paredeLeft and k.paredeDown): 
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo+1,ignora) #vai para a posição à direita da ovelha
        #neste tbm
        elif(k.paredeUp and k.paredeRight):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição em cima da ovelha porque fica mais perto da cerca para a ovelha
        #Casos normais de 1 parede
        elif(k.paredeUp or k.paredeDown): 
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-1,ignora) #vai para a posição à esquerda da ovelha porque fica mais perto da cerca se bater para a ovelha
        elif(k.paredeRight or k.paredeLeft):
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora) #vai para a posição abaixo da ovelha porque fica mais perto da cerca para a ovelha
        else:
            caminho_ovelha = algoritmo_A_star(k.numeroCacifo-6,ignora)

    if(HerdingSheep == True and (len(caminho_ovelha) > 3)):
        #print("Caminho < 3",file=stderr)
        lenUp = 100
        lenDown = 100
        lenRight = 100
        lenLeft = 100
        if(sheep not in [32,33,34,35,31]):
            #print("Len Up:", file=stderr)
            caminho_ovelhaUp = algoritmo_A_star(sheep+6,ignora)
            lenUp = len(caminho_ovelhaUp)
            #print(len(caminho_ovelhaUp),file=stderr)
        if(sheep not in [1,2,3,4,5,6]):
            caminho_ovelhaDown = algoritmo_A_star(sheep-6,ignora)
            lenDown = len(caminho_ovelhaDown)
            #print("Len Down:", file=stderr)
            #print(len(caminho_ovelhaDown),file=stderr)
        if(sheep not in [1,7,13,19,25]):
            caminho_ovelhaLeft = algoritmo_A_star(sheep-1,ignora)
            lenLeft = len(caminho_ovelhaLeft)
            #print("Len Left:", file=stderr)
            #print(len(caminho_ovelhaLeft),file=stderr)
        if(sheep not in [6,12,18,24,30]):
            caminho_ovelhaRight = algoritmo_A_star(sheep+1,ignora)
            lenRight = len(caminho_ovelhaRight)
            #print("Len Right:", file=stderr)
            #print(len(caminho_ovelhaRight),file=stderr)
      
        if(sheep == 1):
            if(len(caminho_ovelhaUp) < 4):
                return caminho_ovelhaUp
            else:
                return caminho_ovelhaRight

        if(sheep == 6):
            if(len(caminho_ovelhaUp) < 4):
                return caminho_ovelhaUp
            else:
                return caminho_ovelhaLeft
        
        if(sheep == 31):
            if(len(caminho_ovelhaDown) < 4):
                return caminho_ovelhaDown
            else:
                return caminho_ovelhaRight
        if(sheep not in [32,33,34,35] and k.paredeUp == False):
            #print("testeUp",file=stderr)
            if(lenUp < 4):
                #print("Len Up:", file=stderr)
                #print(len(caminho_ovelhaUp),file=stderr)
                return caminho_ovelhaUp
        if(sheep not in [2,3,4,5] and k.paredeDown == False):
            #print("testeDown",file=stderr)
            if(lenDown < 4):
                #print("Len Down:", file=stderr)
                #print(len(caminho_ovelhaDown),file=stderr)
                return caminho_ovelhaDown
        if(sheep not in [7,13,19,25] and k.paredeLeft == False):
            #print("testeLeft",file=stderr)
            if(lenLeft < 4):
                #print("Len Left:", file=stderr)
                #print(len(caminho_ovelhaLeft),file=stderr)
                return caminho_ovelhaLeft
        if(sheep not in [30,24,18,12] and k.paredeRight == False):
            #print("testeRight",file=stderr)
            if(lenRight < 4):
                #print("Len Right:", file=stderr)
                #print(len(caminho_ovelhaRight),file=stderr)
                return caminho_ovelhaRight
          
    #print("Caminho > 3",file=stderr)
    return caminho_ovelha        

        
def escolhe_ovelha(): #função que escolhe qual é a melhor ovelha para começar a ser guiada (para evitar ter de fazer mais ifs na função calcula_inicio() )
    ov1 = CacifoAtual(posicao_ovelhas[0]) #1ª ovelha encontrada
    ov2 = CacifoAtual(posicao_ovelhas[1]) #2º ovelha encontrada
    if (ov1.numeroCacifo == ov2.numeroCacifo+1 or ov1.numeroCacifo == ov2.numeroCacifo+6 or ov1.numeroCacifo == ov2.numeroCacifo-1 or ov1.numeroCacifo == ov2.numeroCacifo-6): #Verifica se a 2ª ovelha está num cacifo adjacente ao da 1ª ovelha   
        if(ov1.numeroCacifo in [35,34,33,32,31,25,19,13,7,2,3,4,5,6,12,18,24,30]): #Verifica se a 1ª ovelha está na borda do tabuleiro
            return posicao_ovelhas[1]  #Devolve a 2ª ovelha porque é a mais fácil para começar a guiar
     #caso as ovelhas estejam em cacifos adjacentes ao cacifo do robot
        #neste casos á esquerda do robo
        elif(ov1.numeroCacifo == ov2.numeroCacifo+7):
            #se tem parede em baixo dificulta o trajeto do robot
            if(ov2.paredeDown):
                #entao escolhe a 1 ovelha
                return posicao_ovelhas[0]
            #se tiver parede a direita (ou seja à esquerda do robot)
            elif(ov1.paredeRight):
                #escolhe a ovelha 2 
                return posicao_ovelhas[1]
        elif(ov1.numeroCacifo == ov2.numeroCacifo-7):
            #se tem parede em baixo dificulta o trajeto do robot
            if(ov1.paredeDown):
                #entao escolhe a 2 ovelha
                return posicao_ovelhas[1]
            #se tiver parede a direita (ou seja à esquerda do robot)
            elif(ov2.paredeRight):
                #escolhe a ovelha 2 
                return posicao_ovelhas[0]
        #caso esteja no cacifo adjacente à direita do robot
        elif(ov1.numeroCacifo == ov2.numeroCacifo+5):
            if(ov2.paredeDown):
                return posicao_ovelhas[0]
            elif(ov1.paredeLeft):
                return posicao_ovelhas[1]
        elif(ov1.numeroCacifo == ov2.numeroCacifo-5):
            if(ov1.paredeDown):
                return posicao_ovelhas[1]
            elif(ov2.paredeLeft):
                return posicao_ovelhas[0]
    return posicao_ovelhas[0] #Devolve a 1ª ovelha por motivo nenhum senão simplesmente ser a primeira que foi encontrada


def vai_ate_ovelha(ovelha,HerdingSheep):
    print("ANDOU",file=stderr)
    caminho = calcula_inicio(ovelha,False,HerdingSheep)
    print("LEN CAMINHO DO ANDOU", file=stderr)
    print(len(caminho),file=stderr)
    #print("Caminho:",file=stderr)
    #for i in caminho:
        #print(i.numeroCacifo, file =stderr)
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

    adiciona_parede(7)
    adiciona_parede(10)
    informacao.direcao=90
    adiciona_parede(12)
    adiciona_parede(21)
    adiciona_parede(32)
    adiciona_parede(35)
    
    informacao.direcao=0
    informacao.posicao=14
    guarda_posicao_ovelha()
    informacao.posicao=15
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
        vai_ate_ovelha(betty,False)
        posicaoOvelha_Sendo_Guiada = betty
        while(posicaoOvelha_Sendo_Guiada != 36):
            posicaoOvelha_Sendo_Guiada = interage_ovelha(posicaoOvelha_Sendo_Guiada)
            #print("CICLO MAIN", file=stderr)
            if(posicaoOvelha_Sendo_Guiada != 36):
                vai_ate_ovelha(posicaoOvelha_Sendo_Guiada,True)
            print(posicaoOvelha_Sendo_Guiada,file=stderr)
                

        vai_ate_ovelha(vitoria,False)
        posicaoOvelha_Sendo_Guiada = vitoria
        while(posicaoOvelha_Sendo_Guiada != 36):
            posicaoOvelha_Sendo_Guiada = interage_ovelha(posicaoOvelha_Sendo_Guiada)
            if(posicaoOvelha_Sendo_Guiada != 36):
                vai_ate_ovelha(posicaoOvelha_Sendo_Guiada,True)
            print(posicaoOvelha_Sendo_Guiada,file=stderr)
               
                
    elif (posicao_ovelhas[0] == posicao_ovelhas[1]):
        vai_ate_ovelha(betty,False)
        posicaoOvelha_Sendo_Guiada = betty
        while(posicaoOvelha_Sendo_Guiada != 36):
            posicaoOvelha_Sendo_Guiada = interage_ovelha(posicaoOvelha_Sendo_Guiada)
            if(posicaoOvelha_Sendo_Guiada != 36):
                vai_ate_ovelha(posicaoOvelha_Sendo_Guiada,True)
            print(posicaoOvelha_Sendo_Guiada,file=stderr)
                

    else:
        vitoria = posicao_ovelhas[0]
        vai_ate_ovelha(betty,False)
        posicaoOvelha_Sendo_Guiada = betty
        while(posicaoOvelha_Sendo_Guiada != 36):
            posicaoOvelha_Sendo_Guiada = interage_ovelha(posicaoOvelha_Sendo_Guiada)
            if(posicaoOvelha_Sendo_Guiada != 36):
                vai_ate_ovelha(posicaoOvelha_Sendo_Guiada,True)
            print(posicaoOvelha_Sendo_Guiada,file=stderr)
                

        vai_ate_ovelha(vitoria,False)
        posicaoOvelha_Sendo_Guiada = vitoria
        while(posicaoOvelha_Sendo_Guiada != 36):
            posicaoOvelha_Sendo_Guiada = interage_ovelha(posicaoOvelha_Sendo_Guiada)
            if(posicaoOvelha_Sendo_Guiada != 36):
                vai_ate_ovelha(posicaoOvelha_Sendo_Guiada,True)
            print(posicaoOvelha_Sendo_Guiada,file=stderr)
                
        

if (__name__ == "__main__"):
    main() 

            



