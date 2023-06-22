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
        self.fitness = 0 

    def crearCirculos(self,NCirculos,lower_bound, upper_bound,pixels):
        for i in range(NCirculos):
            circulo = Solution()
            circulo.X = np.random.uniform(lower_bound, upper_bound,3)
            circulo.V = np.random.uniform(lower_bound, upper_bound,3)
            circulo.fitness = objetivo_figura(circulo,pixels[i])
            self.fitness += circulo.fitness
            self.Circulos.append(circulo)
    
    def impcirculos(self):
        for cir in self.Circulos:
            cir.impcirculo()
        print(self.fitness)

# Crea un cumulo inicial en forma aleatoria
def initialize_swarm(size,Ncirculos,lower_bound,upper_bound,pixels):
    swarm = []
    for i in range(size):
        individuo = Imagen()
        individuo.crearCirculos(Ncirculos,lower_bound,upper_bound,pixels)
        swarm.append(individuo)
    return swarm

def best_solution(swarm):
    best_fitness = float('inf') #fitness = -float('inf')
    best_particle = swarm[0] #best = swarm[0]
    for particle in swarm: #particle antes sw
        if particle.fitness < best_fitness: # si fuera maximizar sw.fitness > fitness
            best_particle = particle
            best_fitness = particle.fitness #antes sw.fitness
    return best_particle



image = Image.open("imagenes/imagen2.bmp")
pixels = np.array(image)
print("Image shape:", pixels.shape)
ancho, alto = pixels.shape
print('ancho', ancho, 'alto', alto)
mostrar(pixels,ancho,alto)
cadena = segmentar(pixels,ancho,alto)
lista = corregir(cadena,pixels)
borrarCirculo(lista,pixels)
mostrar(pixels,ancho,alto)
cadena2 = segmentar(pixels,ancho,alto)
lista2 = corregir(cadena2,pixels)
borrarCirculo(lista2,pixels)
mostrar(pixels,ancho,alto)
cadena3 = segmentar(pixels,ancho,alto)
lista3 = corregir(cadena3,pixels)
borrarCirculo(lista3,pixels)
mostrar(pixels,ancho,alto)
circulosLista = []
circulosLista.append(lista)
circulosLista.append(lista2)
circulosLista.append(lista3)

ncirculos = len(circulosLista)
lower_bound = [0,0,0]
upper_bound = [100,50,25]

swarm = initialize_swarm(50,ncirculos,lower_bound,upper_bound,circulosLista)
y = swarm[:]
y_best = copy.copy(best_solution(swarm))
w = 0.5
c1 = 0.2 # intenta regresar
c2 = 0.3 # se acerca al lider
it = 0
while it < 1000:
    #print(y_best.fitness)
    for particle in swarm:
        auxfitness = 0
        print(particle.fitness)
        auxparticle = copy.copy(swarm[swarm.index(particle)])
        for individuo in auxparticle.Circulos:
            for i in range(len(individuo.X)):
                r1 = np.random.rand() # uniform random number r1
                r2 = np.random.rand() # uniform random number r2
                individuo.V[i] = w * individuo.V[i] + c1 * r1 * (y[swarm.index(particle)].Circulos[particle.Circulos.index(individuo)].X[i] - individuo.X[i]) + c2 * r2 * (y_best.Circulos[particle.Circulos.index(individuo)].X[i] - individuo.X[i])
                #Actualizar la posicion de particle.X
                individuo.X[i] = individuo.X[i] + individuo.V[i]
            individuo.validate_bounds(lower_bound, upper_bound)
            #Evaluar la nueva posicion de X
            individuo.fitness = objetivo_figura(individuo,circulosLista[auxparticle.Circulos.index(individuo)])
            auxfitness += individuo.fitness
        auxparticle.fitness = auxfitness
        if auxparticle.fitness <= y[swarm.index(particle)].fitness:
            y[swarm.index(particle)] = auxparticle
        if auxparticle.fitness <= y_best.fitness:
            print(str(auxparticle.fitness)+" <= "+str(y_best.fitness))
            y_best = copy.copy(auxparticle)
        swarm[swarm.index(particle)]=copy.copy(auxparticle)
    it +=1

fig = plt.imshow(image, cmap='gray')
circle1 = plt.Circle((y_best.Circulos[0].X[0], y_best.Circulos[0].X[1]), y_best.Circulos[0].X[2], color='r', fill=False)
circle2 = plt.Circle((y_best.Circulos[1].X[0], y_best.Circulos[1].X[1]), y_best.Circulos[2].X[2], color='r', fill=False)
circle3 = plt.Circle((y_best.Circulos[2].X[0], y_best.Circulos[2].X[1]), y_best.Circulos[2].X[2], color='r', fill=False)
plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.savefig("Hola.png")

