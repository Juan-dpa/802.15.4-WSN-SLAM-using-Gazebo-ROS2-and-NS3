import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv

def plot_csv_map(csv_filename):
    fig, ax = plt.subplots(figsize=(10, 10))

    try:
        with open(csv_filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                # Ignorar líneas vacías o la cabecera
                if not row or row[0].startswith('#'): 
                    continue
                
                # Extraer coordenadas
                xmin, xmax, ymin, ymax, zmin, zmax = map(float, row[:6])

                width = xmax - xmin
                height = ymax - ymin

                # Color: Azul para muros desde el suelo, Naranja para dinteles flotantes
                color = '#1f77b4' if zmin == 0.0 else '#ff7f0e'
                # Transparencia para poder ver superposiciones
                alpha = 0.5 if zmin == 0.0 else 0.8 

                # Dibujar el rectángulo
                rect = patches.Rectangle((xmin, ymin), width, height, 
                                         linewidth=1, edgecolor='black', 
                                         facecolor=color, alpha=alpha)
                ax.add_patch(rect)
                
    except FileNotFoundError:
        print(f"No se encontró el archivo: {csv_filename}")
        return

    # Ajustar ejes para que no se deforme la perspectiva (1 metro X = 1 metro Y)
    ax.autoscale()
    ax.set_aspect('equal')
    
    # Decoración del gráfico
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.title('Vista Planta 2D - Obstáculos exportados a NS-3\n(Azul: Suelo | Naranja: Flotante)')
    plt.xlabel('Eje X (metros)')
    plt.ylabel('Eje Y (metros)')
    
    # Mostrar el gráfico interactivo
    plt.show()

# Lanzar el visualizador
plot_csv_map('buildings.csv')