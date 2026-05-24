#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd
from sklearn.metrics import mutual_info_score
import re


NAT_TO_BIT = 1.0 / np.log(2.0)


def entropy_bits(labels: pd.Series) -> float:
    return mutual_info_score(labels, labels) * NAT_TO_BIT


def mi_bits(x_labels: pd.Series, r_labels: pd.Series) -> float:
    return mutual_info_score(x_labels, r_labels) * NAT_TO_BIT


def build_position_labels(pos_df: pd.DataFrame, cell_size: float, include_z: bool) -> pd.Series:
    ix = np.floor(pos_df["x"].to_numpy(dtype=float) / cell_size).astype(int)
    iy = np.floor(pos_df["y"].to_numpy(dtype=float) / cell_size).astype(int)
    if include_z:
        iz = np.floor(pos_df["z"].to_numpy(dtype=float) / cell_size).astype(int)
        return pd.Series([f"{a}_{b}_{c}" for a, b, c in zip(ix, iy, iz)], index=pos_df.index)
    return pd.Series([f"{a}_{b}" for a, b in zip(ix, iy)], index=pos_df.index)


def bin_rssi_values(values: pd.DataFrame, bin_db: float) -> pd.DataFrame:
    arr = values.to_numpy(dtype=float)
    valid = np.isfinite(arr)
    if not np.any(valid):
        raise ValueError("No hay valores RSSI validos para discretizar.")
    rssi_min = np.floor(np.nanmin(arr) / bin_db) * bin_db
    binned = np.full(arr.shape, -9999, dtype=int)  # categoria para faltantes
    binned[valid] = np.floor((arr[valid] - rssi_min) / bin_db).astype(int)
    return pd.DataFrame(binned, index=values.index, columns=values.columns)


def normalize_name(name: str) -> str:
    return re.sub(r"[^a-z0-9]", "", str(name).strip().lower())


def canonicalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    renamed = {}
    for col in df.columns:
        key = normalize_name(col)
        if key in {"seqnum"}:
            renamed[col] = "Seq_Num"
        elif key in {"times", "time", "timestamp"}:
            renamed[col] = "Time_s"
        elif key in {"x", "xm"}:
            renamed[col] = "x"
        elif key in {"y", "ym"}:
            renamed[col] = "y"
        elif key in {"z", "zm"}:
            renamed[col] = "z"
        elif key in {"srcmac"}:
            renamed[col] = "Src_MAC"
        elif key in {"rssidbm", "rssi"}:
            renamed[col] = "RSSI_dBm"
    return df.rename(columns=renamed)


def load_position_table(path: str) -> pd.DataFrame:
    # 1) Try normal CSV with header, ignoring comment lines.
    pos = pd.read_csv(path, comment="#")
    pos = canonicalize_columns(pos)

    # Case A: explicit Seq_Num format
    if {"Seq_Num", "x", "y", "z"}.issubset(pos.columns):
        out = pos[["Seq_Num", "x", "y", "z"]].copy()
        return out

    # Case B: trajectory format Time_s,X,Y,Z
    if {"Time_s", "x", "y", "z"}.issubset(pos.columns):
        out = pos[["Time_s", "x", "y", "z"]].copy()
        out = out.rename(columns={"Time_s": "Seq_Num"})
        return out

    # 2) Fallback: header-less 4-column numeric file.
    pos_raw = pd.read_csv(path, comment="#", header=None, names=["c0", "c1", "c2", "c3"])
    out = pd.DataFrame(
        {
            "Seq_Num": pos_raw["c0"],
            "x": pos_raw["c1"],
            "y": pos_raw["c2"],
            "z": pos_raw["c3"],
        }
    )
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Calcula informacion mutua entre posicion y vector RSSI.")
    parser.add_argument("--rssi_csv", required=True, help="CSV RSSI: Time_s,Src_MAC,Seq_Num,RSSI_dBm")
    parser.add_argument("--pos_csv", required=True, help="CSV posicion: Seq_Num,x,y,z")
    parser.add_argument("--cell_size", type=float, default=5.0, help="Tamano de celda espacial en metros")
    parser.add_argument("--rssi_bin_db", type=float, default=5.0, help="Ancho de bin RSSI en dB")
    parser.add_argument(
        "--macs",
        nargs="+",
        default=["00:01", "00:02", "00:03"],
        help="Lista de transmisores (Src_MAC) para construir el vector RSSI",
    )
    parser.add_argument("--include_z", action="store_true", help="Discretizar posicion en 3D (x,y,z)")
    args = parser.parse_args()

    rssi = pd.read_csv(args.rssi_csv, comment="#")
    rssi = canonicalize_columns(rssi)
    pos = load_position_table(args.pos_csv)

    needed_rssi_cols = {"Src_MAC", "Seq_Num", "RSSI_dBm"}
    needed_pos_cols = {"Seq_Num", "x", "y", "z"}
    if not needed_rssi_cols.issubset(rssi.columns):
        raise ValueError(f"Faltan columnas en RSSI CSV. Necesarias: {sorted(needed_rssi_cols)}")
    if not needed_pos_cols.issubset(pos.columns):
        raise ValueError(f"Faltan columnas en posiciones CSV. Necesarias: {sorted(needed_pos_cols)}")

    pos = pos.copy()
    rssi = rssi.copy()
    pos["Seq_Num"] = pd.to_numeric(pos["Seq_Num"], errors="coerce")
    pos["x"] = pd.to_numeric(pos["x"], errors="coerce")
    pos["y"] = pd.to_numeric(pos["y"], errors="coerce")
    pos["z"] = pd.to_numeric(pos["z"], errors="coerce")
    rssi["Seq_Num"] = pd.to_numeric(rssi["Seq_Num"], errors="coerce")
    rssi["RSSI_dBm"] = pd.to_numeric(rssi["RSSI_dBm"], errors="coerce")
    pos = pos.dropna(subset=["Seq_Num", "x", "y", "z"])
    rssi = rssi.dropna(subset=["Seq_Num", "Src_MAC", "RSSI_dBm"])

    rssi = rssi[rssi["Src_MAC"].isin(args.macs)].copy()
    if rssi.empty:
        raise ValueError("No hay filas RSSI para las MACs seleccionadas.")

    # Reconstruye vector RSSI por Seq_Num (promedio si hay duplicados)
    rssi_vec = (
        rssi.pivot_table(index="Seq_Num", columns="Src_MAC", values="RSSI_dBm", aggfunc="mean")
        .reindex(columns=args.macs)
        .sort_index()
    )

    merged = pos.merge(rssi_vec, how="inner", left_on="Seq_Num", right_index=True).sort_values("Seq_Num")
    if merged.empty:
        raise ValueError("No hay interseccion de Seq_Num entre posicion y RSSI.")

    x_labels = build_position_labels(merged, cell_size=args.cell_size, include_z=args.include_z)

    rssi_binned = bin_rssi_values(merged[args.macs], bin_db=args.rssi_bin_db)
    r_labels = pd.Series(
        [",".join(map(str, row)) for row in rssi_binned.to_numpy(dtype=int)],
        index=rssi_binned.index,
    )

    h_x = entropy_bits(x_labels)
    h_r = entropy_bits(r_labels)
    i_xr = mi_bits(x_labels, r_labels)
    h_x_given_r = max(h_x - i_xr, 0.0)

    i_norm = (i_xr / h_x) if h_x > 0 else np.nan
    n_eff_initial = 2.0 ** h_x
    n_eff_after = 2.0 ** h_x_given_r
    ambiguity_reduction_factor = 2.0 ** i_xr

    print("=== Mutual Information RSSI -> Position ===")
    print(f"Muestras usadas: {len(merged)}")
    print(f"MACs: {args.macs}")
    print(f"Cell size: {args.cell_size:.2f} m | RSSI bin: {args.rssi_bin_db:.2f} dB")
    print()
    print(f"H(X): {h_x:.4f} bits")
    print(f"H(R): {h_r:.4f} bits")
    print(f"H(X|R): {h_x_given_r:.4f} bits")
    print(f"I(X;R): {i_xr:.4f} bits")
    print(f"I(X;R)/H(X): {i_norm:.4f}" if np.isfinite(i_norm) else "I(X;R)/H(X): NaN")
    print(f"2^H(X): {n_eff_initial:.4f} celdas efectivas")
    print(f"2^H(X|R): {n_eff_after:.4f} celdas efectivas")
    print(f"2^I(X;R): {ambiguity_reduction_factor:.4f} factor de reduccion")
    print()

    pct = 100.0 * i_norm if np.isfinite(i_norm) else np.nan
    if np.isfinite(pct):
        print(
            "Interpretacion: El RSSI reduce la incertidumbre espacial en "
            f"{i_xr:.4f} bits, equivalente al {pct:.2f}% de la incertidumbre inicial. "
            f"El numero efectivo de posiciones posibles baja de {n_eff_initial:.2f} "
            f"a {n_eff_after:.2f} celdas, una reduccion media de factor "
            f"{ambiguity_reduction_factor:.2f}."
        )
    else:
        print(
            "Interpretacion: H(X)=0, no hay incertidumbre espacial inicial en la "
            "discretizacion elegida; la informacion mutua no es interpretable."
        )


if __name__ == "__main__":
    main()
