# Cohemos — FAST-LIVO2 para UGV (ROS 2 Humble)

Repositorio: https://github.com/Spaceia3000/FAST-LIVO2-ROS2-MID360-Fisheye (rama **Cohemos**).

Este paquete contiene el frontend LiDAR/IMU/Sony. El backend de cierre de lazo se instala desde https://github.com/Spaceia3000/simple_loop_closure (también rama **Cohemos**). El driver MID-360, la adquisición de imágenes Sony y las librerías Vikit son dependencias externas; no vienen dentro de estos dos repositorios.

## Matriz de dependencias y versiones

Referencias **medidas en la laptop** el 19/10/2026. Ubuntu 22.04.5 LTS x86_64, ROS 2 Humble, Python 3.10.12, GCC 11.4.0, C++17. No son versiones ya validadas en la Jetson ARM64.

| Librería o repositorio | Versión/commit observado | Fuente |
|---|---|---|
| ROS 2 Humble | ros-humble-ros-base **0.10.0** | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html |
| rclcpp | **16.0.21** | Paquete ros-humble-rclcpp |
| OpenCV / cv_bridge | **4.5.4 / 3.2.1** | libopencv-dev / ros-humble-cv-bridge |
| PCL / pcl_ros | **1.12.1 / 2.4.5** | libpcl-dev / ros-humble-pcl-ros |
| Eigen / Boost / fmt | **3.4.0 / 1.74 / 8.1.1** | libeigen3-dev / libboost-dev / libfmt-dev |
| Sophus | **1.22.9102** | ros-humble-sophus, CMake desde /opt/ros/humble |
| image_transport | **3.1.13** | ros-humble-image-transport |
| livox_ros_driver2 | **1.2.6**, Git **13eb05e4e6dd7a765b934d0c5fd6236676a57b49** | https://github.com/Spaceia3000/livox_ros_driver2 (rama dev/mid360-integration) |
| Livox-SDK2 | Bibliotecas SDK presentes en /usr/local/lib; **versión no identificada** | https://github.com/Livox-SDK/Livox-SDK2 |
| vikit_common + vikit_ros | Los paquetes instalados indican 0.0.0, **no versión upstream válida**; fuente de referencia Git **0b5548d9adab58128f3a59a507e96a83acfa8fdf** | https://github.com/Rhymer-Lcy/rpg_vikit_ros2_fisheye |
| FAST-LIVO2 | Rama Cohemos creada desde integration/v1-humble @ **8faf006** | Este repositorio |

**Sobre Vikit:** las carpetas locales del workspace FAST no contienen su propia historia Git; el commit citado es del repositorio fuente disponible en la laptop, no una comprobación byte por byte de esas carpetas.

**Sobre Livox-SDK2:** elegir la versión compatible con el driver siguiendo sus instrucciones oficiales y registrar después su commit. No se ha verificado el SHA de la instalación actual.

## Instalación en Ubuntu 22.04 x86_64

Instalar ROS 2 Humble desde la guía oficial anterior; después:

~~~bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libfmt-dev \
  ros-humble-ament-cmake ros-humble-rosidl-default-generators \
  ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-visualization-msgs \
  ros-humble-sophus ros-humble-rviz2
~~~

1. Instalar Livox-SDK2 desde https://github.com/Livox-SDK/Livox-SDK2.
2. Instalar el fork de livox_ros_driver2 fijado a su commit:

~~~bash
mkdir -p ~/Documents/sensors_ws/src
cd ~/Documents/sensors_ws/src
git clone https://github.com/Spaceia3000/livox_ros_driver2.git
git -C livox_ros_driver2 checkout 13eb05e4e6dd7a765b934d0c5fd6236676a57b49
~~~

Completar la compilación del driver **para ROS 2** según su README y generar ~/Documents/sensors_ws/install/setup.bash. El LiDAR debe publicar el tipo Livox CustomMsg.

3. Instalar las dos librerías Vikit desde esta fuente de referencia:

~~~bash
git clone https://github.com/Rhymer-Lcy/rpg_vikit_ros2_fisheye.git ~/Documents/rpg_vikit_ros2_fisheye
git -C ~/Documents/rpg_vikit_ros2_fisheye checkout 0b5548d9adab58128f3a59a507e96a83acfa8fdf
~~~

Colocar vikit_common y vikit_ros como paquetes hermanos de fast_livo dentro del **mismo workspace**. Esta versión de CMakeLists.txt enlaza las bibliotecas Vikit desde la carpeta install de ese workspace, por lo que no deben quedarse solo en una instalación externa arbitraria.

4. Clonar FAST-LIVO2 Cohemos:

~~~bash
mkdir -p ~/Documents/fast_livo2_baseline_ws/src
cd ~/Documents/fast_livo2_baseline_ws/src
git clone --single-branch --branch Cohemos \
  https://github.com/Spaceia3000/FAST-LIVO2-ROS2-MID360-Fisheye.git
~~~

Con las dependencias instaladas y Vikit en ese mismo src:

~~~bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
cd ~/Documents/fast_livo2_baseline_ws
colcon build --packages-select vikit_common vikit_ros --symlink-install
source install/setup.bash
colcon build --packages-select fast_livo --symlink-install
source install/setup.bash
~~~

**No necesitas GTSAM para FAST-LIVO2**; pertenece al backend Simple Loop Closure.

## Archivos que incluye esta rama

- src/, include/, msg/: código C++, estimación y mensajes ROS 2.
- config/cohemo_lio.yaml: MID-360 + IMU, sin cámara.
- config/cohemo_livo.yaml: MID-360 + IMU + Sony.
- config/camera_sony_ev9520l.yaml: intrínsecos/distorsión registrados.
- launch/mapping_ugv.launch.py: único launch para el robot.
- rviz_cfg/mapping_ugv.rviz: visualizador opcional.
- Log/: directorios de salida editables; ROOT_DIR se fija al compilar.
- CMakeLists.txt, package.xml y LICENSE.

**No incluye un driver Sony:** debes proporcionar por separado un nodo de adquisición/capturadora que publique sensor_msgs/Image en /cam0/image_raw. La capturadora, el zoom y los timestamps hardware deben ser compatibles con la calibración.

## Arranque con robot real

Iniciar sensores y cámara primero. Configurar todos los nodos con el mismo dominio ROS y descubrimiento DDS compatible.

~~~bash
source /opt/ros/humble/setup.bash
source ~/Documents/sensors_ws/install/setup.bash
source ~/Documents/fast_livo2_baseline_ws/install/setup.bash

ros2 launch fast_livo mapping_ugv.launch.py \
  mode:=cohemo_livo.yaml \
  camera_params_file:=camera_sony_ev9520l.yaml \
  use_sim_time:=false \
  use_rviz:=false
~~~

Para solo LiDAR+IMU: mode:=cohemo_lio.yaml. El frontend proporciona /aft_mapped_to_init y /cloud_registered_metric con el mismo timestamp para el backend ExactTime; /cloud_registered aporta nube RGB local.

Para reproducir un rosbag, utilizar use_sim_time:=true y ros2 bag play --clock. ROS_LOCALHOST_ONLY=1 es apropiado solo si todos los nodos deben permanecer en un mismo equipo; **no usarlo en un UGV con sensores repartidos entre varias máquinas**.

## Jetson AGX Orin y límites

No copiar binarios x86_64 a ARM64. Verificar versión real de JetPack/L4T y compatibilidad con Ubuntu, ROS Humble, Livox-SDK2, SDK de cámara, Sophus y Vikit; recompilar para ARM64 y registrar versiones/SHAs. Este README describe las versiones de la laptop, **no un despliegue ya validado en Orin**.

FAST-LIVO2 LIVO todavía puede quedar esperando ante huecos de imágenes Sony. El fallback LIO continuo no está certificado. Las calibraciones son específicas del montaje; no se acredita navegación segura ni F3. Pruebas y evidencia histórica están en integration/v1-humble.
