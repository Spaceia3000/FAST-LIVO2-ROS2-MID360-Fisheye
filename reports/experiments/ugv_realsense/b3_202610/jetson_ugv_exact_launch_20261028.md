# Arranque UGV — FAST-LIVO2 / SimpleLoopClosure — 2026-10-28

**Puerta de aceptación física: PENDIENTE.** Validado el contrato de los launchers y
los paquetes instalados en Jetson AGX Orin. No se ha ejecutado un recorrido físico.

## Versiones y frontera de datos

- Código funcional de Jetson en `ws_fast_livo2`: `ugv_lsi_camera_gap@15a5ad8`.
- Código de laptop/GitHub: `ugv_lsi_camera_gap@1a28b1d` (diferencia **solo documental**).
- Backend SimpleLoopClosure: `ugv_lsi@c272c28`.
- Vikit: `fix/jetson-humble-ament-vikit@d218bd3`.
- Guardar TODOS los informes, resultados, gráficas, capturas y archivos de evidencia en
  la Lenovo: `~/Documents/fast_livo2_validation_reports/2026-10-28_ugv_b3_acceptance/`.
- La Jetson debe mantener exclusivamente los workspaces instalados y sus
  directorios normales de build/install/log. No crear rosbag de prueba ni reportes
  adicionales ni alterar archivos de calibración o RViz personalizados.
- Abrir **una terminal distinta** por cada proceso. Los overlays se cargan
  automáticamente con el `.bashrc` actual de la Jetson.

## Terminal 1: Livox MID-360 CustomMsg (si no está arrancado)

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

El archivo de launch instalado fija `namespace='mid360'`. Esto produce los
tópicos esperados `/mid360/livox/lidar` y `/mid360/livox/imu`. El JSON del
MID-360 apunta al sensor `192.168.168.10` y al host `192.168.168.163`.
**No cambiar las IP ni iniciar un segundo driver Livox en paralelo.**

**No usar** `ros2 launch ugv_fastlivo2_launch ugv_robot.launch.py` junto
con la Terminal 1: ese launch general ya inicia un driver Livox sin namespace,
lo que podría producir tópicos `/livox/*` incompatibles con los YAML actuales,
y además duplicaría los sensores.

## Terminal 2: RealSense D435i (solo necesaria para LIVO)

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera serial_no:=_207122078450 config_file:=$(ros2 pkg prefix ugv_fastlivo2_launch)/share/ugv_fastlivo2_launch/config/realsense_d435i_640x480_fastlivo2.yaml
```

YAML existente: RGB 640 × 480 a 30 Hz; depth e IMU de RealSense
desactivados. FAST usa la IMU Livox. Comprobar:

```bash
ros2 topic list
ros2 topic hz /mid360/livox/lidar
ros2 topic hz /mid360/livox/imu
ros2 topic hz /camera/camera/color/image_raw
```

Cada `ros2 topic hz` es diagnóstico interactivo; detenerlo con Ctrl+C.

## Terminal 3: FAST-LIVO2, elegir UNA modalidad

```bash
# LO: LiDAR únicamente
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_lo.yaml use_sim_time:=false use_rviz:=true
```

```bash
# LIO: LiDAR + IMU
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_lio.yaml use_sim_time:=false use_rviz:=true
```

```bash
# LIVO referencia: LiDAR + IMU + RealSense, 10 Hz y 0.10 m
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_livo.yaml use_sim_time:=false use_rviz:=true
```

```bash
# LIVO B3 candidato: LiDAR + IMU + RealSense, 7.5 Hz y 0.15 m
ros2 launch fast_livo mapping_ugv.launch.py mode:=ugv_realsense_livo.yaml use_sim_time:=false use_rviz:=true
```

Ejecutar únicamente **uno**; parar con Ctrl+C antes de cambiar de modalidad.
`use_rviz:=true` requiere sesión gráfica Jetson y aumenta la carga de GPU.
Durante la prueba de estabilidad prolongada usar `use_rviz:=false` si no se
necesita visualización local. La configuración `ugv_livo.rviz` contiene ya
los displays `/pgo_rgb_map_cloud`, `/simple_loop_closure/optimized_keyframes`
y `/simple_loop_closure/loop_edges`. Son datos que llegan solo con backend
de cierre de lazo activo.

## Terminal 4: SimpleLoopClosure, únicamente LIO/LIVO

```bash
# Para FAST-LIVO2 LIO:
ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=lio
```

```bash
# Para FAST-LIVO2 LIVO referencia o B3:
ros2 launch simple_loop_closure simple_loop_closure.launch.py mode:=livo
```

**No se ha implementado LO + SimpleLoopClosure en este contrato**.
El cierre de lazo requiere `/aft_mapped_to_init` y
`/cloud_registered_metric` sincronizados por stamp. Para verificar:

```bash
ros2 topic list
ros2 topic info /cloud_registered_metric
ros2 topic info /aft_mapped_to_init
ros2 topic info /pgo_rgb_map_cloud
```

El hecho de que el mapa RGB aparezca **no prueba** un cierre aceptado.
Confirmar aristas y correcciones de pose, además de métricas de lazo y mapas.

## RViz en Lenovo (alternativa a la GPU Jetson)

En una terminal de Lenovo, con ROS 2 Humble cargado:

```bash
rviz2 -d ~/Documents/fast_livo2_baseline_ws/src/FAST-LIVO2-ROS2-MID360-Fisheye/rviz_cfg/ugv_livo.rviz
```

**Requisito previo:** Lenovo y Jetson deben tener conectividad DDS ROS 2
(enrutamiento/segmento de red apropiado, misma configuración de dominio y
sin restricción localhost). En la auditoría, Lenovo estaba en
`10.101.148.226/17`, Jetson Wi-Fi en `172.20.10.10/28` y Jetson
Ethernet en `192.168.168.163/24`: la accesibilidad DDS desde Lenovo
**no está demostrada**. Primero comprobar `ros2 topic list` en Lenovo;
si no aparecen los tópicos, utilizar la visualización en la Jetson,
sin asumir descubrimiento ROS a través de acceso remoto.

## Criterios de aceptación física y promoción

Comprobar cada modalidad sin pérdidas de tracking, sin divergencia, TF consistente,
nube RGB alineada y valores de frecuencia/latencia, CPU/RAM y colas del estimador
registrados en Lenovo. Comparar referencia LIVO 10 Hz/0.10 m frente a B3
7.5 Hz/0.15 m en misma ruta/condiciones; completar 60+ minutos **en movimiento**
sin crecimiento no acotado de backlog ni memoria. Para Loop Closure, comprobar
visita repetida, grafo, aristas aceptadas y corrección métrica, no solo RViz.

**No hacer merge a `integration/v1-humble` sin validación física y
aprobación expresa del operador.**
