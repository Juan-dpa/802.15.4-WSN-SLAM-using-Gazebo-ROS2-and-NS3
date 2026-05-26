import xml.etree.ElementTree as ET
import argparse
import csv
import math
from pathlib import Path

import trimesh


SCRIPT_DIR = Path(__file__).resolve().parent

# ns3::Building::BuildingType_t
BUILDING_RESIDENTIAL = 0
BUILDING_OFFICE = 1
BUILDING_COMMERCIAL = 2

# ns3::Building::ExtWallsType_t
WALL_WOOD = 0
WALL_CONCRETE_WITH_WINDOWS = 1
WALL_CONCRETE_WITHOUT_WINDOWS = 2
WALL_STONE_BLOCKS = 3

VEGETATION_KEYWORDS = (
    "vegetation",
    "tree",
    "bush",
    "shrub",
    "plant",
    "crop",
    "row",
    "arbusto",
    "arbol",
)

def parse_pose(pose_elem):
    """Extrae [x, y, z, roll, pitch, yaw] desde un elemento XML."""
    if pose_elem is not None and pose_elem.text:
        return list(map(float, pose_elem.text.strip().split()))
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def resolve_mesh_path(uri):
    """Resuelve URIs de Gazebo/SDF contra el directorio del script."""
    uri = uri.strip()
    candidates = []

    if uri.startswith("file://"):
        candidates.append(Path(uri.replace("file://", "", 1)))
    elif uri.startswith("model://"):
        model_path = Path(uri.replace("model://", "", 1))
        candidates.extend([
            SCRIPT_DIR / model_path,
            SCRIPT_DIR / "models" / model_path,
            SCRIPT_DIR / "meshes" / model_path.name,
        ])
    else:
        mesh_path = Path(uri)
        candidates.extend([
            mesh_path,
            SCRIPT_DIR / mesh_path,
            SCRIPT_DIR / "meshes" / mesh_path.name,
        ])

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None

def get_mesh_box(uri):
    """Calcula el Bounding Box de un archivo .dae o .stl usando trimesh."""
    mesh_path = resolve_mesh_path(uri)
    if mesh_path is not None:
        try:
            mesh = trimesh.load(mesh_path, force='mesh')
            return mesh.bounds[1] - mesh.bounds[0]
        except ImportError as exc:
            print(f"Aviso: no se pudo leer {mesh_path.name}: {exc}. Uso caja por defecto.")
        except Exception as exc:
            print(f"Aviso: error leyendo {mesh_path.name}: {exc}. Uso caja por defecto.")
    return [0.5, 0.5, 0.5] # Valor por defecto si no encuentra la malla

def parse_geometry_size(geom):
    """Devuelve una caja equivalente [sx, sy, sz] para la geometría SDF."""
    box_elem = geom.find('box/size')
    if box_elem is not None and box_elem.text:
        return list(map(float, box_elem.text.strip().split())), "box"

    sphere_elem = geom.find('sphere/radius')
    if sphere_elem is not None and sphere_elem.text:
        radius = float(sphere_elem.text.strip())
        diameter = 2.0 * radius
        return [diameter, diameter, diameter], "sphere"

    cylinder_radius = geom.find('cylinder/radius')
    cylinder_length = geom.find('cylinder/length')
    if (
        cylinder_radius is not None and cylinder_radius.text
        and cylinder_length is not None and cylinder_length.text
    ):
        radius = float(cylinder_radius.text.strip())
        length = float(cylinder_length.text.strip())
        return [2.0 * radius, 2.0 * radius, length], "cylinder"

    mesh_elem = geom.find('mesh/uri')
    if mesh_elem is not None and mesh_elem.text:
        sx, sy, sz = get_mesh_box(mesh_elem.text)

        scale_elem = geom.find('mesh/scale')
        if scale_elem is not None and scale_elem.text:
            scale_x, scale_y, scale_z = map(float, scale_elem.text.strip().split())
            sx *= scale_x
            sy *= scale_y
            sz *= scale_z

        return [sx, sy, sz], "mesh"

    return None

def get_named_node(path, tag):
    return next((node for node in path if node.tag == tag), None)

def get_path_names(path):
    return " ".join(node.get("name", "").lower() for node in path)

def has_green_material(*nodes):
    for node in nodes:
        if node is None:
            continue

        material_elems = node.findall(".//material/diffuse")
        material_elems.extend(node.findall(".//material/ambient"))
        for elem in material_elems:
            if elem.text is None:
                continue
            try:
                red, green, blue, *_ = map(float, elem.text.strip().split())
            except ValueError:
                continue
            if green > 0.3 and green > red * 1.2 and green > blue * 1.2:
                return True
    return False

def classify_collision(path, geometry_kind):
    """Mapea cada colisión a los enums de ns3::Building."""
    names = get_path_names(path)
    link_node = get_named_node(path, "link")
    model_node = get_named_node(path, "model")

    is_vegetation = (
        geometry_kind == "sphere"
        or any(keyword in names for keyword in VEGETATION_KEYWORDS)
        or has_green_material(link_node, model_node)
    )

    if is_vegetation:
        return BUILDING_COMMERCIAL, WALL_WOOD

    if "wall" in names or "building" in names:
        return BUILDING_OFFICE, WALL_CONCRETE_WITH_WINDOWS

    return BUILDING_OFFICE, WALL_WOOD

def extract_boxes(root, writer):
    parent_map = {c: p for p in root.iter() for c in p}
    written = 0
    skipped = 0

    for col in root.findall('.//collision'):
        geom = col.find('geometry')
        if geom is None:
            skipped += 1
            continue

        size = parse_geometry_size(geom)
        if size is None:
            skipped += 1
            continue
        (sx, sy, sz), geometry_kind = size

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
            
        b_type, w_type = classify_collision(path, geometry_kind)

        # Filtro de minicajas
        if (max(xs) - min(xs)) < 0.01 and (max(ys) - min(ys)) < 0.01:
            continue

        writer.writerow([round(min(xs), 3), round(max(xs), 3), 
                         round(min(ys), 3), round(max(ys), 3), 
                         round(max(0, global_z - sz/2), 3), round(global_z + sz/2, 3), b_type, w_type])
        written += 1

    return written, skipped

def default_input():
    for filename in ("agriculture_world.sdf", "model.sdf"):
        path = SCRIPT_DIR / filename
        if path.exists():
            return path
    return SCRIPT_DIR / "model.sdf"

def parse_args():
    parser = argparse.ArgumentParser(description="Extrae bounding boxes de colisiones SDF a CSV.")
    parser.add_argument("sdf", nargs="?", type=Path, default=default_input())
    parser.add_argument("-o", "--output", type=Path)
    return parser.parse_args()

def main():
    args = parse_args()
    sdf_path = args.sdf if args.sdf.is_absolute() else SCRIPT_DIR / args.sdf
    if args.output:
        output_path = args.output
    elif sdf_path.name == "agriculture_world.sdf":
        output_path = SCRIPT_DIR / "agriculture.csv"
    else:
        output_path = sdf_path.with_suffix(".csv")
    output_path = output_path if output_path.is_absolute() else SCRIPT_DIR / output_path

    tree = ET.parse(sdf_path)
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['# xMin', 'xMax', 'yMin', 'yMax', 'zMin', 'zMax', 'Type', 'Wall'])
        written, skipped = extract_boxes(tree.getroot(), writer)

    print(f"CSV generado: {output_path}")
    print(f"Colisiones exportadas: {written}; omitidas: {skipped}")

if __name__ == "__main__":
    main()
