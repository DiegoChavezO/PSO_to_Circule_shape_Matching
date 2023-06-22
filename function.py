import math

def objetivo_figura(circulo, pixels):
    center_x, center_y, radius = circulo.X
    total_distance = 0
    for pixel in pixels:
        x = pixel[1]
        y = pixel[0]
        # formula DISTANCIA EUCLIDEANA
        distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2) #duda en el radio
        # Si el punto es negro, sumar su distancia al total
        total_distance += abs(distance - radius)
    return total_distance