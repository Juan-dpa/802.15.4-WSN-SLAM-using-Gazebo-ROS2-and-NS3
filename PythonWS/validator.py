import argparse
import csv
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

if not os.environ.get("DISPLAY"):
    import matplotlib

    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Patch, Rectangle


SCRIPT_DIR = Path(__file__).resolve().parent

TYPE_NAMES = {
    0: "Residential",
    1: "Office",
    2: "Commercial / vegetacion",
}

WALL_NAMES = {
    0: "Wood",
    1: "ConcreteWithWindows",
    2: "ConcreteWithoutWindows",
    3: "StoneBlocks",
}

STYLES = {
    (2, 0): {
        "label": "Vegetacion (Type=2, Wall=0)",
        "facecolor": "#4f9d45",
        "edgecolor": "#255f28",
        "alpha": 0.42,
        "zorder": 2,
    },
    (1, 1): {
        "label": "Edificio / muro (Type=1, Wall=1)",
        "facecolor": "#8f8f8f",
        "edgecolor": "#303030",
        "alpha": 0.62,
        "zorder": 4,
    },
    (1, 0): {
        "label": "Objeto ligero (Type=1, Wall=0)",
        "facecolor": "#d6b66a",
        "edgecolor": "#7d6427",
        "alpha": 0.55,
        "zorder": 3,
    },
}

DEFAULT_STYLE = {
    "label": "Otro",
    "facecolor": "#6baed6",
    "edgecolor": "#24537a",
    "alpha": 0.55,
    "zorder": 3,
}

POSITION_MARKERS = ["*", "o", "s", "^", "D", "P"]


def resolve_path(path):
    path = Path(path)
    return path if path.is_absolute() else SCRIPT_DIR / path


def read_boxes(csv_filename):
    boxes = []
    with open(csv_filename, "r", newline="") as f:
        reader = csv.reader(f)
        for line_number, row in enumerate(reader, start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 8:
                print(f"Aviso: fila {line_number} ignorada, columnas insuficientes: {row}")
                continue

            try:
                xmin, xmax, ymin, ymax, zmin, zmax = map(float, row[:6])
                building_type = int(float(row[6]))
                wall_type = int(float(row[7]))
            except ValueError:
                print(f"Aviso: fila {line_number} ignorada, valores no numericos: {row}")
                continue

            boxes.append(
                {
                    "xmin": xmin,
                    "xmax": xmax,
                    "ymin": ymin,
                    "ymax": ymax,
                    "zmin": zmin,
                    "zmax": zmax,
                    "type": building_type,
                    "wall": wall_type,
                }
            )
    return boxes


def read_positions(csv_filename):
    positions = []
    with open(csv_filename, "r", newline="") as f:
        reader = csv.reader(f)
        for line_number, row in enumerate(reader, start=1):
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 3:
                print(f"Aviso: posicion {line_number} ignorada, columnas insuficientes: {row}")
                continue

            try:
                x, y, z = map(float, row[:3])
            except ValueError:
                print(f"Aviso: posicion {line_number} ignorada, valores no numericos: {row}")
                continue

            positions.append({"x": x, "y": y, "z": z})
    return positions


def get_style(box):
    return STYLES.get((box["type"], box["wall"]), DEFAULT_STYLE)


def add_box(ax, box):
    style = get_style(box)
    width = box["xmax"] - box["xmin"]
    height = box["ymax"] - box["ymin"]

    rect = Rectangle(
        (box["xmin"], box["ymin"]),
        width,
        height,
        facecolor=style["facecolor"],
        edgecolor=style["edgecolor"],
        linewidth=0.8,
        alpha=style["alpha"],
        zorder=style["zorder"],
    )
    ax.add_patch(rect)


def point_inside_box(position, box, include_z=True):
    inside_xy = (
        box["xmin"] <= position["x"] <= box["xmax"]
        and box["ymin"] <= position["y"] <= box["ymax"]
    )
    if not include_z:
        return inside_xy
    return inside_xy and box["zmin"] <= position["z"] <= box["zmax"]


def add_positions(ax, positions, boxes):
    labels = ["Coordinator", "ED1", "ED2", "ED3"]
    for index, position in enumerate(positions):
        inside_xyz = any(point_inside_box(position, box, include_z=True) for box in boxes)
        inside_xy = any(point_inside_box(position, box, include_z=False) for box in boxes)
        marker = POSITION_MARKERS[index % len(POSITION_MARKERS)]
        color = "#d62728" if inside_xyz else "#ff7f0e" if inside_xy else "#1f77b4"
        label = labels[index] if index < len(labels) else f"Node {index}"

        ax.scatter(
            position["x"],
            position["y"],
            marker=marker,
            s=130 if index == 0 else 82,
            c=color,
            edgecolors="black",
            linewidths=0.8,
            zorder=10,
        )
        ax.annotate(
            label,
            (position["x"], position["y"]),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
            color="#111111",
            zorder=11,
        )


def set_limits(ax, boxes, positions=None):
    xs = [box["xmin"] for box in boxes] + [box["xmax"] for box in boxes]
    ys = [box["ymin"] for box in boxes] + [box["ymax"] for box in boxes]
    if positions:
        xs.extend(position["x"] for position in positions)
        ys.extend(position["y"] for position in positions)

    xmin = min(xs)
    xmax = max(xs)
    ymin = min(ys)
    ymax = max(ys)

    span = max(xmax - xmin, ymax - ymin)
    margin = max(span * 0.04, 1.0)

    ax.set_xlim(xmin - margin, xmax + margin)
    ax.set_ylim(ymin - margin, ymax + margin)
    ax.set_aspect("equal", adjustable="box")


def make_legend(ax, boxes, positions=None):
    used_keys = []
    for box in boxes:
        key = (box["type"], box["wall"])
        if key not in used_keys:
            used_keys.append(key)

    handles = []
    for key in used_keys:
        style = STYLES.get(key, DEFAULT_STYLE)
        type_name = TYPE_NAMES.get(key[0], f"Type={key[0]}")
        wall_name = WALL_NAMES.get(key[1], f"Wall={key[1]}")
        label = style["label"]
        if style is DEFAULT_STYLE:
            label = f"{type_name}, {wall_name} ({key[0]},{key[1]})"
        handles.append(
            Patch(
                facecolor=style["facecolor"],
                edgecolor=style["edgecolor"],
                alpha=style["alpha"],
                label=label,
            )
        )

    if positions:
        handles.extend(
            [
                Patch(facecolor="#1f77b4", edgecolor="black", label="Nodo libre"),
                Patch(facecolor="#ff7f0e", edgecolor="black", label="Nodo sobre obstaculo en XY"),
                Patch(facecolor="#d62728", edgecolor="black", label="Nodo dentro de volumen"),
            ]
        )

    ax.legend(handles=handles, loc="upper right", framealpha=0.94)


def print_summary(boxes):
    counts = {}
    for box in boxes:
        key = (box["type"], box["wall"])
        counts[key] = counts.get(key, 0) + 1

    print(f"Obstaculos cargados: {len(boxes)}")
    for key, count in sorted(counts.items()):
        type_name = TYPE_NAMES.get(key[0], f"Type={key[0]}")
        wall_name = WALL_NAMES.get(key[1], f"Wall={key[1]}")
        print(f"  Type={key[0]} ({type_name}), Wall={key[1]} ({wall_name}): {count}")


def print_position_summary(positions, boxes):
    if not positions:
        return

    labels = ["Coordinator", "ED1", "ED2", "ED3"]
    print(f"Posiciones cargadas: {len(positions)}")
    for index, position in enumerate(positions):
        label = labels[index] if index < len(labels) else f"Node {index}"
        inside_xyz = [
            box for box in boxes if point_inside_box(position, box, include_z=True)
        ]
        inside_xy = [
            box for box in boxes if point_inside_box(position, box, include_z=False)
        ]
        status = "libre"
        if inside_xyz:
            status = "DENTRO de volumen"
        elif inside_xy:
            status = "encima en XY, fuera por Z"
        print(
            f"  {label}: ({position['x']}, {position['y']}, {position['z']}) -> "
            f"{status}; solapes XY={len(inside_xy)}, solapes XYZ={len(inside_xyz)}"
        )


def plot_2d_boxes(csv_filename, output=None, show=False, positions_file=None):
    csv_path = resolve_path(csv_filename)
    boxes = read_boxes(csv_path)
    if not boxes:
        print(f"No hay obstaculos validos en {csv_path}")
        return

    positions = read_positions(resolve_path(positions_file)) if positions_file else []

    print_summary(boxes)
    print_position_summary(positions, boxes)

    fig, ax = plt.subplots(figsize=(11, 9))

    for box in sorted(boxes, key=lambda item: get_style(item)["zorder"]):
        add_box(ax, box)

    if positions:
        add_positions(ax, positions, boxes)

    set_limits(ax, boxes, positions=positions)
    make_legend(ax, boxes, positions=positions)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Validacion 2D de obstaculos: {csv_path.name}")
    ax.grid(True, color="#d6d6d6", linewidth=0.6, alpha=0.75)

    fig.tight_layout()

    output_path = resolve_path(output) if output else csv_path.with_suffix(".png")
    fig.savefig(output_path, dpi=180)
    print(f"Figura guardada: {output_path}")

    if show:
        plt.show()

    plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(description="Valida en 2D un CSV de obstaculos para ns-3.")
    parser.add_argument("csv", nargs="?", default="agriculture.csv")
    parser.add_argument("-o", "--output", help="Ruta de la imagen PNG de salida.")
    parser.add_argument("--positions", help="CSV de posiciones estaticas X,Y,Z para superponer nodos.")
    parser.add_argument("--show", action="store_true", help="Muestra la ventana de matplotlib.")
    return parser.parse_args()


def main():
    args = parse_args()
    plot_2d_boxes(args.csv, output=args.output, show=args.show, positions_file=args.positions)


if __name__ == "__main__":
    main()
