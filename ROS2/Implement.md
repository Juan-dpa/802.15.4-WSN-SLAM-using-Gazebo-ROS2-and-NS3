# RSSI SLAM Replay en ROS2/RViz

Guia breve para visualizar el algoritmo Range-Only SLAM con datos CSV.

## Docker

El contenedor debe tener GTSAM disponible para Python:

```bash
python3 -c "import rclpy; import gtsam; import numpy; print('OK')"
```

Si falla `gtsam`, instalarlo en el Dockerfile antes de `USER rssa`:

```dockerfile
RUN python3 -m pip install --no-cache-dir --break-system-packages \
    numpy pandas scipy scikit-learn matplotlib gtsam
```

## Ejecucion

Desde la carpeta donde esten `rssi_slam_node.py`, `Inputs/` y `Outputs/`:

```bash
python3 rssi_slam_node.py --ros-args \
  -p rssi_csv:=Outputs/slam_dataset_run1-synthetic.csv \
  -p odom_csv:=Inputs/trajectory_odom-synthetic.csv
```

Para acelerar el replay:

```bash
python3 rssi_slam_node.py --ros-args \
  -p rssi_csv:=Outputs/slam_dataset_run1-synthetic.csv \
  -p odom_csv:=Inputs/trajectory_odom-synthetic.csv \
  -p odom_rows_per_tick:=5 \
  -p publish_period_s:=0.02
```

## RViz2

Abrir:

```bash
rviz2
```

En `Global Options`:

```text
Fixed Frame = map
```

Anadir displays:

```text
TF
Path            -> Topic: /path_replay
MarkerArray     -> Topic: /ap_markers
PointCloud2     -> Topic: /rssi_heatmap
```

Para ver mapas separados por AP, anadir tambien los topics:

```text
/rssi_heatmap/ap_00_02
/rssi_heatmap/ap_00_03
/rssi_heatmap/ap_00_04
```

Los topics exactos dependen de las MACs presentes en el CSV.

## PointCloud2

Configuracion recomendada para los heatmaps:

```text
Style = Points
Size (m) = 0.5
Color Transformer = Intensity
```

## Comprobaciones

Ver topics publicados:

```bash
ros2 topic list
```

Estado del algoritmo:

```bash
ros2 topic echo /rssi_slam_status
```

El replay publica la trayectoria odometrica del CSV. La ground truth no se usa como entrada del algoritmo.
