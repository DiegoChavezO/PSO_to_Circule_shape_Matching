'''
    Proyecto final
    Elaborado por:
        -Diego Chávez Ortiz
        -Denzel Omar Vázquez Pérez
'''
from PIL import Image
import numpy as np
import copy
import random
import matplotlib.pyplot as plt
from segmentacion import *
from function import *

class Solution:
    def __init__(self): #aqui comienzo a modificar todo
        self.X = []#vector decision donde cada lista dentro es un circulos
        self.V = []
        self.fitness = 0

    def set_fitness(self, value):
        self.fitness = value

    def validate_bounds(self, lower_bound, upper_bound):
        for i in range(len(self.X)):
            if self.X[i] < lower_bound[i] or self.X[i] > upper_bound[i]:
                self.X[i] = random.uniform(lower_bound[i], upper_bound[i])
    
    def impcirculo(self):
        print(str(self.X)+"-"+str(self.V)+"-"+str(self.fitness))

#Clase para N-Ciculos como individuo
class Imagen:
    def __init__(self):
        self.Circulos=[]

    def crearCirculos(self,NCirculos,lower_bound, upper_bound,pixels):
        for i in range(NCirculos):
            circulo = Solution()
            circulo.X = np.random.uniform(lower_bound, upper_bound,3)
            circulo.V = np.random.uniform(lower_bound, upper_bound,3)
            circulo.set_fitness(objetivo_figura(circulo,pixels[i]))
            self.Circulos.append(circulo)
    
    def impcirculos(self):
        for cir in self.Circulos:
            cir.impcirculo()

# Crea un cumulo inicial en forma aleatoria
def initialize_swarm(size,Ncirculos,lower_bound,upper_bound,pixels):
    swarm = []
    for i in range(size):
        individuo = Imagen()
        individuo.crearCirculos(Ncirculos,lower_bound,upper_bound,pixels)
        swarm.append(individuo)
    return swarm

def best_solution(swarm):
    best_fitness = [float('inf')]*len(swarm[0].Circulos)
    best_particle = copy.copy(swarm[0])
    for particle in swarm: #particle antes sw
        for i in range(len(particle.Circulos)):
            if particle.Circulos[i].fitness < best_fitness[i]: # si fuera maximizar sw.fitness > fitness
                best_particle.Circulos[i] = particle.Circulos[i]
                best_fitness[i] = particle.Circulos[i].fitness #antes sw.fitness
    return best_particle

def segmentarcirculos(pixels,ancho,alto):
    circulosLista = []
    while(True):
        cadena = segmentar(pixels,ancho,alto)
        if(len(cadena)==0):
            break
        puntos = delinear(cadena)
        lista = corregir(cadena,pixels)
        borrarCirculo(lista,pixels)
        circulosLista.append(puntos)
    return circulosLista

def algoritmoPSO(nparticulas,iteraciones,w,c1,c2):
    swarm = initialize_swarm(nparticulas,ncirculos,lower_bound,upper_bound,circulosLista)
    y = swarm[:]
    y_best = copy.deepcopy(best_solution(swarm))
    it = 0
    while it < iteraciones:
        for particle in swarm:
            auxparticle = copy.deepcopy(particle)
            for individuo in auxparticle.Circulos:
                for i in range(len(individuo.X)):
                    r1 = np.random.rand() # uniform random number r1
                    r2 = np.random.rand() # uniform random number r2
                    individuo.V[i] = w * individuo.V[i] + c1 * r1 * (y[swarm.index(particle)].Circulos[auxparticle.Circulos.index(individuo)].X[i] - individuo.X[i]) + c2 * r2 * (y_best.Circulos[auxparticle.Circulos.index(individuo)].X[i]- individuo.X[i])
                    #Actualizar la posicion de particle.X
                    individuo.X[i] = individuo.X[i] + individuo.V[i]
                individuo.validate_bounds(lower_bound, upper_bound)
                #Evaluar la nueva posicion de X
                individuo.fitness = objetivo_figura(individuo,circulosLista[auxparticle.Circulos.index(individuo)])
                if individuo.fitness <= y[swarm.index(particle)].Circulos[auxparticle.Circulos.index(individuo)].fitness:
                    y[swarm.index(particle)].Circulos[auxparticle.Circulos.index(individuo)].fitness = individuo.fitness
                if individuo.fitness <= y_best.Circulos[auxparticle.Circulos.index(individuo)].fitness:
                    y_best.Circulos[auxparticle.Circulos.index(individuo)] = copy.deepcopy(individuo)
            particle=copy.deepcopy(auxparticle)
        it +=1
    return y_best

if __name__ == "__main__":
    image = Image.open("imagenes/imagen2_incompleta.bmp")
    pixels = np.array(image)
    ancho, alto = pixels.shape
    circulosLista = segmentarcirculos(pixels,ancho,alto)
    ncirculos = len(circulosLista)
    lower_bound = [0,0,0]
    upper_bound = [100,50,25]
    w = 0.7
    c1 = 0.2 # intenta regresar
    c2 = 0.3 # se acerca al lider
    y_best = algoritmoPSO(100,10000,w,c1,c2)
    print("###########################################################")
    print("Mejor individuo final: ")
    y_best.impcirculos()
    print("###########################################################")
    fig = plt.imshow(image, cmap='gray')
    circulosplot = []
    for i in range(len(circulosLista)):
        circle = plt.Circle((y_best.Circulos[i].X[0], y_best.Circulos[i].X[1]), y_best.Circulos[i].X[2], color='r', fill=False)
        circulosplot.append(circle)
    for c in circulosplot:
        plt.gca().add_patch(c)
    plt.savefig("resultados.png")
