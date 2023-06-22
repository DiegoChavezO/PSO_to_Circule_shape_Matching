'''from PIL import Image
import numpy as np
import math
import copy'''

def segmentar(image_data,ancho,alto):
	contador=0
	cadena = []
	dx=[0,0]
	dy=[0,100]
	auxdx=[0,0]
	auxdy=[0,100]
	dx2 = [0,0]
	dy2 = [0,0]
	var = True
	for i in range(0,ancho):
		auxdx = dx.copy()
		auxdy = dy.copy()
		aux=[]
		if(var==True):
			dx2 = dx.copy()
			dy2 = dy.copy()
		for j in range(0, alto):
			pixel_value = image_data[i, j]
			if(pixel_value!=254):
				aux.append([i,j])
			nj=j+1
			if(nj<alto):
				if(pixel_value==254 and image_data[i,nj]!=254):
					if contador==0:
						dx=[i,nj]
					contador+=1
				elif(contador==2 and image_data[i,nj]==254):
					dy=[i,j]
					if(dx2[1]+2<=dx[1] and dy2[1]<dy[1]):
						var = False
					else:
						cadena.append(aux)
						var = True
					contador=0
					break
		contador=0
	return cadena

def borrarCirculo(lista,image_data):
	for i in lista:
		image_data[i[0],i[1]]=254

def corregir(cadena,image_data):
	cadlar = 0
	for i in range(len(cadena)):
		if(cadena[i][len(cadena[i])-1][1]>cadlar):
			cadlar = cadena[i][len(cadena[i])-1][1]
	limite=[cadena[len(cadena)-1][0][0],cadlar]
	lista = [elemento for sublista in cadena for elemento in sublista]
	contador = 0 
	while(True):
		if(contador==len(lista)):
			break
		if(lista[contador][1]>limite[1]):
			lista.remove(lista[contador])
		else:
			contador+=1

	for i in range(0,limite[0]):
		for j in range(0,limite[1]):
			image_data[i,j]=254
	return lista

def mostrar(image_data,ancho,alto):
	for i in range(0,ancho):
		for j in range(0,alto):
			if(image_data[i,j]==254):
				print("-",end="")
			else:
				print(0,end="")
		print()

'''# Open the BMP image file
image = Image.open("imagenes/imagen1.bmp")

# Get the image data as a numpy array
image_data = np.array(image)
# Display the shape of the image data
print("Image shape:", image_data.shape)
ancho, alto = image_data.shape
print('ancho', ancho, 'alto', alto)

cadena = segmentar(image_data)
print(cadena)
mostrar(image_data)
lista = corregir(cadena,image_data)
print(lista)
borrarCirculo(lista,image_data)
mostrar(image_data)'''
