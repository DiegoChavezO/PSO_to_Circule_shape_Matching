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



image = Image.open("imagenes/imagen2_incompleta.bmp")
pixels = np.array(image)
print("Image shape:", pixels.shape)
ancho, alto = pixels.shape
print('ancho', ancho, 'alto', alto)
cadena = segmentar(pixels,ancho,alto)
puntos1 = delinear(cadena)
lista = corregir(cadena,pixels)
borrarCirculo(lista,pixels)
cadena2 = segmentar(pixels,ancho,alto)
puntos2 = delinear(cadena2)
lista2 = corregir(cadena2,pixels)
borrarCirculo(lista2,pixels)
cadena3 = segmentar(pixels,ancho,alto)
mostrar(pixels,ancho,alto)
puntos3 = delinear(cadena3)
print(puntos3)
lista3 = corregir(cadena3,pixels)
borrarCirculo(lista3,pixels)
mostrar(pixels,ancho,alto)
circulosLista = []
circulosLista.append(puntos1)
circulosLista.append(puntos2)
circulosLista.append(puntos3)

'''ncirculos = len(circulosLista)
lower_bound = [0,0,0]
upper_bound = [100,50,25]

swarm = initialize_swarm(50,ncirculos,lower_bound,upper_bound,circulosLista)
y = swarm[:]
y_best = copy.deepcopy(best_solution(swarm))
w = 0.7
c1 = 0.2 # intenta regresar
c2 = 0.3 # se acerca al lider
it = 0
while it < 10000:
    print("###########################################################")
    print("Mejor individuo: ")
    y_best.impcirculos()
    print("###########################################################\n")
    for particle in swarm:
        auxparticle = copy.deepcopy(particle)
        for individuo in auxparticle.Circulos:
            for i in range(len(individuo.X)):
                r1 = np.random.rand() # uniform random number r1
                r2 = np.random.rand() # uniform random number r2
                individuo.V[i] = w * individuo.V[i] + c1 * r1 * (y[swarm.index(particle)].Circulos[auxparticle.Circulos.index(individuo)].X[i] - individuo.X[i]) + c2 * r2 * (y_best.Circulos[auxparticle.Circulos.index(individuo)].X[i]- individuo.X[i])
                #individuo.V[i] = w * individuo.V[i] + c1 * r1 * (y_best.Circulos[auxparticle.Circulos.index(individuo)].X[i]- individuo.X[i]) + c2 * r2 * (individuo.X[i] - individuo.X[i])
                #Actualizar la posicion de particle.X
                individuo.X[i] = individuo.X[i] + individuo.V[i]
            individuo.validate_bounds(lower_bound, upper_bound)
            #Evaluar la nueva posicion de X
            individuo.fitness = objetivo_figura(individuo,circulosLista[auxparticle.Circulos.index(individuo)])
            if individuo.fitness <= y[swarm.index(particle)].Circulos[auxparticle.Circulos.index(individuo)].fitness:
                y[swarm.index(particle)].Circulos[auxparticle.Circulos.index(individuo)].fitness = individuo.fitness
            if individuo.fitness <= y_best.Circulos[auxparticle.Circulos.index(individuo)].fitness:
                #print(str(individuo.fitness)+" <= "+str(y_best.Circulos[auxparticle.Circulos.index(individuo)].fitness))
                y_best.Circulos[auxparticle.Circulos.index(individuo)] = copy.deepcopy(individuo)
        particle=copy.deepcopy(auxparticle)
    plt.scatter(y_best.Circulos[auxparticle.Circulos.index(individuo)].X[0], y_best.Circulos[auxparticle.Circulos.index(individuo)].X[1], color='r', s=5)
    it +=1
print("###########################################################")
print("Mejor individuo final: ")
y_best.impcirculos()
print("###########################################################")
fig = plt.imshow(image, cmap='gray')
circle1 = plt.Circle((y_best.Circulos[0].X[0], y_best.Circulos[0].X[1]), y_best.Circulos[0].X[2], color='r', fill=False)
circle2 = plt.Circle((y_best.Circulos[1].X[0], y_best.Circulos[1].X[1]), y_best.Circulos[2].X[2], color='r', fill=False)
circle3 = plt.Circle((y_best.Circulos[2].X[0], y_best.Circulos[2].X[1]), y_best.Circulos[2].X[2], color='r', fill=False)
plt.gca().add_patch(circle1)
plt.gca().add_patch(circle2)
plt.gca().add_patch(circle3)
plt.savefig("Hola3.png")
plt.savefig("Hola5.png")'''
