# Cohemos — FAST-LIVO2 para UGV (ROS 2 Humble)

Rama Cohemos derivada de integration/v1-humble. Fuente del frontend
LiDAR/IMU/Sony; el proyecto del lazo cerrado Simple Loop Closure se distribuye
separadamente en su propia rama Cohemos.

## Archivos incluidos

- src/, include/: estimador FAST-LIVO2, filtros, sensores y telemetria.
- msg/: interfaces ROS 2 utilizadas por el estimador.
- config/cohemo_lio.yaml: LiDAR MID-360 + IMU.
- config/cohemo_livo.yaml: MID-360 + IMU + camara Sony.
- config/camera_sony_ev9520l.yaml: intrinsecos/distorsion de la Sony.
- launch/mapping_ugv.launch.py: unico launch con perfiles LIO/LIVO.
- rviz_cfg/mapping_ugv.rviz: vista base para mapeo.
- Log/: directorio de salidas que el estimador crea durante su ejecucion.
- CMakeLists.txt, package.xml, LICENSE: compilacion y licencia.

No se incluyen datasets, tests, documentos de investigacion ni launch
para sensores que el UGV Cohemo no utiliza.

## Dependencias ROS 2 Humble

Ubuntu 22.04, livox_ros_driver2 (con CustomMsg), vikit_common, vikit_ros,
OpenCV, PCL, Eigen, Sophus, fmt, Boost, GTSAM solo para el backend SLC,
y las dependencias ROS 2 declaradas en package.xml.

El directorio Log/ debe ser escribible y la ruta de construccion
ROOT_DIR se establece al compilar, por lo que no hay que mover/eliminar
el arbol de fuente despues de construir.

## Compilacion

Dentro de un workspace ROS2 que tenga previamente instalados los paquetes
Livox y Vikit y con las dependencias disponibles:

~~~bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
cd ~/Documents/fast_livo2_baseline_ws
colcon build --packages-select fast_livo --symlink-install
source install/setup.bash
~~~

## Modo LIVO con robot real

Inicia primero los drivers MID-360, IMU y Sony sincronizados y disponibles
en /mid360/livox/lidar, /mid360/livox/imu y /cam0/image_raw.

~~~bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
source ~/Documents/fast_livo2_baseline_ws/install/setup.bash
ros2 launch fast_livo mapping_ugv.launch.py   mode:=cohemo_livo.yaml   camera_params_file:=camera_sony_ev9520l.yaml   use_sim_time:=false   use_rviz:=false
~~~

Para LIO sin camara, cambiar mode:=cohemo_lio.yaml.
El frontend publica /aft_mapped_to_init y /cloud_registered_metric
con timestamps iguales para el sincronizador ExactTime de SLC;
la nube RGB para colorear el mapa global se publica en /cloud_registered.

Para reproducir un rosbag en lugar de usar sensores reales:
use_sim_time:=true y publicar /clock en el mismo ROS_DOMAIN_ID.

## Seguridad y limites

La calibracion guardada y la correccion temporal Cohemo son especificas
de la instalacion y los registros previos; se deben verificar al cambiar
montajes o hardware. LIVO puede quedarse esperando imagen Sony durante
huecos reales de captura. No se ha certificado un fallback continuo a LIO
en este frontend. El cierre geometrico cientifico F3 de SLC sigue abierto.

Los tests, calibraciones historicas e informes reproducibles se conservan
en integration/v1-humble, que esta rama no modifica.
