import xml.etree.ElementTree as ET
import csv
import math
import trimesh
import os

def parse_pose(pose_elem):
    """Extrae [x, y, z, roll, pitch, yaw] desde un elemento XML."""
    if pose_elem is not None and pose_elem.text:
        return list(map(float, pose_elem.text.strip().split()))
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def get_mesh_box(uri):
    """Calcula el Bounding Box de un archivo .dae o .stl usando trimesh."""
    # Ajusta la ruta base según dónde estén tus mallas relativas al script
    mesh_path = uri.replace("model://turtlebot3_house/", "./meshes/") 
    if os.path.exists(mesh_path):
        mesh = trimesh.load(mesh_path, force='mesh')
        # Obtenemos los límites (bounds) del objeto en su sistema local
        return mesh.bounds[1] - mesh.bounds[0] # Tamaño [sx, sy, sz]
    return [0.5, 0.5, 0.5] # Valor por defecto si no encuentra la malla

def extract_boxes(root, writer):
    parent_map = {c: p for p in root.iter() for c in p}

    for col in root.findall('.//collision'):
        geom = col.find('geometry')
        if geom is None: continue
            
        # 1. Determinar dimensiones y posibles Escalas
        box_elem = geom.find('box/size')
        mesh_elem = geom.find('mesh/uri')
        scale_elem = geom.find('mesh/scale') # Buscamos si hay escala
        
        # Valor de escala por defecto (X, Y, Z = 1.0)
        scale_x, scale_y, scale_z = 1.0, 1.0, 1.0
        if scale_elem is not None and scale_elem.text:
            scale_x, scale_y, scale_z = map(float, scale_elem.text.strip().split())

        if box_elem is not None:
            sx, sy, sz = map(float, box_elem.text.strip().split())
        elif mesh_elem is not None:
            # Obtenemos las medidas del archivo .dae
            sx_raw, sy_raw, sz_raw = get_mesh_box(mesh_elem.text)
            # ¡Las dividimos por la escala que dice Gazebo!
            sx = sx_raw / scale_x
            sy = sy_raw / scale_y
            sz = sz_raw / scale_z
        else:
            continue

        # Subir por el árbol para sumar poses absolutas
        path = []
        curr = col
        while curr is not None:
            path.append(curr)
            curr = parent_map.get(curr)

        # Cálculo de posición y rotación global acumulada
        global_x, global_y, global_z = 0.0, 0.0, 0.0
        total_yaw = 0.0
        for node in reversed(path):
            pose_elem = node.find('pose')
            tx, ty, tz, _, _, yaw = parse_pose(pose_elem)
            
            global_x += (tx * math.cos(total_yaw) - ty * math.sin(total_yaw))
            global_y += (tx * math.sin(total_yaw) + ty * math.cos(total_yaw))
            global_z += tz
            total_yaw += yaw

        # Calcular límites finales
        dx, dy = sx / 2.0, sy / 2.0
        corners = [(dx, dy), (dx, -dy), (-dx, dy), (-dx, -dy)]
        xs, ys = [], []
        for cx, cy in corners:
            rx = cx * math.cos(total_yaw) - cy * math.sin(total_yaw)
            ry = cx * math.sin(total_yaw) + cy * math.cos(total_yaw)
            xs.append(global_x + rx)
            ys.append(global_y + ry)
            
        # Clasificación
        link_node = next((n for n in path if n.tag == 'link'), None)
        link_name = link_node.get('name', '').lower() if link_node is not None else ''
        
        if 'wall' in link_name:
            b_type, w_type = 1, 1 
        else:
            b_type, w_type = 1, 0 

        # Filtro de minicajas
        if (max(xs) - min(xs)) < 0.01 and (max(ys) - min(ys)) < 0.01:
            continue

        writer.writerow([round(min(xs), 3), round(max(xs), 3), 
                         round(min(ys), 3), round(max(ys), 3), 
                         round(max(0, global_z - sz/2), 3), round(global_z + sz/2, 3), b_type, w_type])

# --- Ejecución ---
tree = ET.parse('model.sdf')
with open('buildings.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['# xMin', 'xMax', 'yMin', 'yMax', 'zMin', 'zMax', 'Type', 'Wall'])
    extract_boxes(tree.getroot(), writer)