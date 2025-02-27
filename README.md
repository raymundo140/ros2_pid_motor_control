# ROS 2 PID Motor Control 🚀

Este proyecto implementa un **controlador PID** en **ROS 2** para regular la velocidad de un motor de corriente continua simulado. Se utilizan **namespaces** para gestionar múltiples instancias del sistema y un archivo de configuración en formato **YAML** para modificar parámetros de control sin necesidad de modificar el código.

## Características
- **Control PID** para regular la velocidad del motor.
- **Generador de referencia** con señales: senoidal, cuadrada y escalón.
- **Modificación de parámetros en tiempo real** con `rqt_reconfigure`.
- **Visualización de datos** con `PlotJuggler`.
- **Estructura modular** con **namespaces** para instancias independientes.
- **Configuración flexible** con archivos `.yaml`.

---

## Instalación

### 1️. Clonar el repositorio
```bash
cd ~/ros2_ws/src
git clone git@github.com:raymundo140/ros2_pid_motor_control.git motor_control
```


### 2. Compilar el paquete
```bash
cd ~/ros2_ws
colcon build --packages-select motor_control
source install/setup.bash
```

### 3. Ejecutar el programa

## 1. Ejecutar los nodos por separado

```bash
ros2 run motor_control controller
ros2 run motor_control dc_motor
ros2 run motor_control set_point
```


## 2. Ejecutar todos los nodos con un archivo de lanzamiento
```bash
ros2 launch motor_control motor_launch.py
```

### 3. Visualizar datos 
```bash
ros2 run plotjuggler plotjuggler
```



