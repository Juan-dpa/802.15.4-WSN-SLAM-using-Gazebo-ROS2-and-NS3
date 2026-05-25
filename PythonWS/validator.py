import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import csv
import numpy as np

def plot_3d_boxes(csv_filename):
    # Inicializar la figura 3D
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    try:
        with open(csv_filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                # Ignorar líneas vacías o la cabecera
                if not row or row[0].startswith('#'): 
                    continue
                
                # Extraer las 6 coordenadas principales
                xmin, xmax, ymin, ymax, zmin, zmax = map(float, row[:6])
                
                # 1. Definir los 8 vértices geométricos del cubo
                vertices = np.array([
                    [xmin, ymin, zmin], [xmax, ymin, zmin], [xmax, ymax, zmin], [xmin, ymax, zmin], # Base
                    [xmin, ymin, zmax], [xmax, ymin, zmax], [xmax, ymax, zmax], [xmin, ymax, zmax]  # Techo
                ])
                
                # 2. Definir las 6 caras uniendo los vértices
                faces = [
                    [vertices[0], vertices[1], vertices[2], vertices[3]], # Abajo
                    [vertices[4], vertices[5], vertices[6], vertices[7]], # Arriba
                    [vertices[0], vertices[1], vertices[5], vertices[4]], # Frente
                    [vertices[2], vertices[3], vertices[7], vertices[6]], # Fondo
                    [vertices[1], vertices[2], vertices[6], vertices[5]], # Derecha
                    [vertices[4], vertices[7], vertices[3], vertices[0]]  # Izquierda
                ]
                
                # 3. Estilo: Distinguir los objetos que flotan
                color = '#1f77b4' if zmin == 0.0 else '#ff7f0e' # Azul (Suelo), Naranja (Flotante)
                alpha = 0.4 if zmin == 0.0 else 0.7
                
                # 4. Añadir el polígono al gráfico
                ax.add_collection3d(Poly3DCollection(
                    faces, facecolors=color, linewidths=0.5, edgecolors='black', alpha=alpha
                ))
                
    except FileNotFoundError:
        print(f"No se encontró el archivo: {csv_filename}")
        return

    # Ajuste CRÍTICO para matplotlib 3D: Forzar los límites para evitar deformación
    # (Ajusta estos números si tu mapa es más grande de 10 metros)
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([0, 4])
    
    # Decoración
    ax.set_xlabel('Eje X (metros)')
    ax.set_ylabel('Eje Y (metros)')
    ax.set_zlabel('Eje Z (Altura)')
    plt.title('Validación 3D de Obstáculos (Gemelo Digital para NS-3)')
    
    # Renderizar
    plt.show()

# Ejecutar con tu archivo
plot_3d_boxes('buildings.csv')