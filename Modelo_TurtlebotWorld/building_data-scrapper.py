import xml.etree.ElementTree as ET
import csv
import math

def parse_pose(pose_elem):
    """Devuelve [x, y, z, roll, pitch, yaw] o ceros si no existe."""
    if pose_elem is not None and pose_elem.text:
        return list(map(float, pose_elem.text.strip().split()))
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def extract_boxes(root, writer):
    # Crear un diccionario para poder "subir" por el árbol (hijo -> padre)
    parent_map = {c: p for p in root.iter() for c in p}

    for col in root.findall('.//collision'):
        geom = col.find('geometry')
        if geom is None:
            continue
            
        box_elem = geom.find('box/size')
        cyl_elem = geom.find('cylinder')
        
        # Determinar el tamaño de la hitbox geométrica
        if box_elem is not None:
            sx, sy, sz = map(float, box_elem.text.strip().split())
        elif cyl_elem is not None:
            # Aproximar cilindro (patas de mesa) a un bounding box
            r = float(cyl_elem.find('radius').text)
            sz = float(cyl_elem.find('length').text)
            sx, sy = r * 2.0, r * 2.0
        else:
            continue # Ignoramos mallas 3D complejas

        # Rastrear la jerarquía hacia arriba (Colision -> Link -> Model -> Root)
        path = []
        curr = col
        while curr is not None:
            path.append(curr)
            curr = parent_map.get(curr)

        # Extraer la altura Z absoluta sumando los offset de los padres
        global_z = 0.0
        for node in path:
            pose_elem = node.find('pose')
            _, _, tz, _, _, _ = parse_pose(pose_elem)
            global_z += tz
            
        zMin = max(0.0, global_z - (sz / 2.0))
        zMax = global_z + (sz / 2.0)

        # Transformar las 4 esquinas del bounding box local a global
        dx, dy = sx / 2.0, sy / 2.0
        local_corners = [(dx, dy), (dx, -dy), (-dx, dy), (-dx, -dy)]
        global_corners_x = []
        global_corners_y = []
        
        for cx, cy in local_corners:
            gx, gy = cx, cy
            # Aplicar traslación y rotación de cada padre (de abajo a arriba)
            for node in path:
                pose_elem = node.find('pose')
                tx, ty, _, _, _, yaw = parse_pose(pose_elem)
                nx = tx + (gx * math.cos(yaw) - gy * math.sin(yaw))
                ny = ty + (gx * math.sin(yaw) + gy * math.cos(yaw))
                gx, gy = nx, ny
            global_corners_x.append(gx)
            global_corners_y.append(gy)
            
        xMin, xMax = min(global_corners_x), max(global_corners_x)
        yMin, yMax = min(global_corners_y), max(global_corners_y)

        # Clasificación para NS-3
        # Subimos al link para ver el nombre y decidir material
        link_node = next((n for n in path if n.tag == 'link'), None)
        link_name = link_node.get('name', '').lower() if link_node is not None else ''
        
        if 'wall' in link_name:
            b_type, w_type = 1, 1 # Office, ConcreteWithWindows
        else:
            b_type, w_type = 1, 0 # Office, Wood (Para mesas y armarios)

        # Filtrar minicajas invisibles (basura del diseño en Gazebo)
        if (xMax - xMin) < 0.01 and (yMax - yMin) < 0.01:
            continue

        writer.writerow([round(xMin, 3), round(xMax, 3), round(yMin, 3), round(yMax, 3), 
                         round(zMin, 3), round(zMax, 3), b_type, w_type])

def main():
    xml_filepath = 'model.sdf'
    csv_filepath = 'buildings.csv'
    
    tree = ET.parse(xml_filepath)
    root = tree.getroot()

    with open(csv_filepath, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['# xMin', 'xMax', 'yMin', 'yMax', 'zMin', 'zMax', 'BuildingType', 'ExtWallType'])
        extract_boxes(root, writer)
        
    print(f"¡Análisis completo! Archivo de obstáculos NS-3 generado: {csv_filepath}")

if __name__ == '__main__':
    main()