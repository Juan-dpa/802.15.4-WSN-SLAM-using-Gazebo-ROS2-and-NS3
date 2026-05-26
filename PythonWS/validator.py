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


def set_limits(ax, boxes):
    xmin = min(box["xmin"] for box in boxes)
    xmax = max(box["xmax"] for box in boxes)
    ymin = min(box["ymin"] for box in boxes)
    ymax = max(box["ymax"] for box in boxes)

    span = max(xmax - xmin, ymax - ymin)
    margin = max(span * 0.04, 1.0)

    ax.set_xlim(xmin - margin, xmax + margin)
    ax.set_ylim(ymin - margin, ymax + margin)
    ax.set_aspect("equal", adjustable="box")


def make_legend(ax, boxes):
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


def plot_2d_boxes(csv_filename, output=None, show=False):
    csv_path = resolve_path(csv_filename)
    boxes = read_boxes(csv_path)
    if not boxes:
        print(f"No hay obstaculos validos en {csv_path}")
        return

    print_summary(boxes)

    fig, ax = plt.subplots(figsize=(11, 9))

    for box in sorted(boxes, key=lambda item: get_style(item)["zorder"]):
        add_box(ax, box)

    set_limits(ax, boxes)
    make_legend(ax, boxes)

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
    parser.add_argument("--show", action="store_true", help="Muestra la ventana de matplotlib.")
    return parser.parse_args()


def main():
    args = parse_args()
    plot_2d_boxes(args.csv, output=args.output, show=args.show)


if __name__ == "__main__":
    main()
