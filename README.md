# Robot móvil Ackermann

Robot móvil con cinemática tipo Ackermann, modelado en Autodesk Fusion 360 y exportado a Xacro/URDF. Se simula en Gazebo Harmonic y se controla con un nodo de ROS 2 (Pure Pursuit) que recorre una trayectoria en forma de infinito (lemniscata de Bernoulli).

**Integrantes:** _Steven Yari_, _Cristhian Aldean_, _Alexa Marin_
**Curso:** Robots Móviles y Articulados, ESPOL

---

## 1. Requisitos

| Componente | Versión |
|---|---|
| Sistema operativo | Ubuntu 24.04 LTS |
| ROS 2 | **Jazzy Jalisco** (desktop) |
| Simulador | **Gazebo Harmonic** (`gz sim`, versión 8.x), instalado con `ros-jazzy-ros-gz` |
| Python | 3.12 (el de Ubuntu 24.04) |

> ROS 2 Humble no está soportado: el plugin Ackermann (`gz-sim-ackermann-steering-system`) y los launch usan Gazebo Harmonic.

### Dependencias

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 python3-colcon-common-extensions python3-rosdep liburdfdom-tools
```

---

## 2. Compilar

```bash
git clone https://github.com/StevenYari/ws_Project_group14.git
cd ws_Project_group14

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

La compilación debe terminar con `Summary: 2 packages finished`.

Comprobación rápida del modelo (opcional):

```bash
xacro src/ackermann_description/urdf/ackermann_bot.xacro > /tmp/r.urdf
check_urdf /tmp/r.urdf      # debe decir "Successfully Parsed XML"
```

---

## 3. Ejecutar la simulación

En **una sola terminal** (con los dos `source` hechos):

```bash
ros2 launch ackermann_description full_sim.launch.py
```

Qué ocurre:

1. Se abre Gazebo con un mundo vacío y aparece el robot (rojo y negro).
2. Se activa el bridge ROS ↔ Gazebo (`/cmd_vel`, `/odom`, `/tf`, `/clock`, `/joint_states`).
3. A los 10 s arranca el controlador y el robot recorre el ∞ (unos 40 s de tiempo simulado).
4. Al terminar se detiene y la terminal imprime una línea como:
   `FIN. Duracion 40.7 s | RMSE cte = 0.064 m | cte max = 0.128 m | CSV: ...`

Gazebo queda abierto hasta que lo cierres con `Ctrl+C`.

Si tu equipo es lento o corre en una máquina virtual, el tiempo real será mayor que el simulado (el recorrido puede tardar cerca de un minuto).

### Argumentos del launch

| Argumento | Valor por defecto | Descripción |
|---|---|---|
| `path_type` | `lemniscate` | `lemniscate` (infinito), `circle` u `oval` |
| `lookahead` | `0.6` | distancia de anticipación L_d del Pure Pursuit [m] |
| `speed` | `0.4` | velocidad lineal constante [m/s] |
| `size` | `3.0` | infinito: semilargo `a` [m]; el radio mínimo de curvatura es `a/3` |
| `radius` | `1.5` | radio del círculo o de las curvas del óvalo [m] |
| `straight` | `2.0` | largo de las rectas del óvalo [m] |
| `delay` | `10.0` | segundos de espera antes de arrancar el controlador |

Ejemplos:

```bash
ros2 launch ackermann_description full_sim.launch.py lookahead:=0.6
ros2 launch ackermann_description full_sim.launch.py path_type:=circle radius:=1.5
ros2 launch ackermann_description full_sim.launch.py speed:=0.3 size:=3.5
```

El radio mínimo de giro del robot es de unos 0.83 m, así que cualquier trayectoria debe tener radio de curvatura mayor. El nodo avisa con un `WARN` si la trayectoria pide más curvatura de la que el robot puede dar.

### Ejecutar simulación y controlador por separado

```bash
# Terminal 1: simulación (Gazebo + robot + bridge)
ros2 launch ackermann_description sim.launch.py

# Terminal 2: controlador
ros2 run ackermann_control pure_pursuit --ros-args -p use_sim_time:=true -p lookahead:=0.6
```

### Mover el robot a mano (prueba rápida)

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"     # detener
```

---

## 4. Estructura del repositorio

```
ws_Project_group14/
├── README.md
├── docs/                         # figuras de resultados
├── tools/plot_results.py         # gráficas y métricas a partir de los CSV
└── src/
    ├── ackermann_description/    # modelo del robot y simulación
    │   ├── urdf/ackermann_bot.xacro
    │   ├── meshes/*.stl          # piezas exportadas de Fusion 360
    │   └── launch/
    │       ├── sim.launch.py         # Gazebo + robot + bridge
    │       └── full_sim.launch.py    # sim + controlador (un solo comando)
    └── ackermann_control/        # controlador
        └── ackermann_control/pure_pursuit.py
```

---

## 5. Robot y modelo

Valores tomados del URDF exportado de Fusion 360.

| Parámetro | Valor |
|---|---|
| Distancia entre ejes L | 0.24 m |
| Ancho de vía W | 0.14 m |
| Distancia entre pivotes | 0.06 m |
| Radio de rueda | 0.04 m |
| Ángulo máximo de dirección (plugin / límite mecánico) | 0.28 rad / 0.314 rad |
| Radio mínimo de giro | ≈ 0.83 m |
| Masa total | ≈ 13.7 kg |
| Tracción / dirección | ruedas traseras / ruedas delanteras |

Notas del modelo:

- El CAD tenía el frente del robot hacia −X. El Xacro lo corrige con un link raíz `base_footprint` girado 180° y con los ejes de los joints ajustados.
- Las colisiones de las ruedas son cilindros, y la fricción de las ruedas es μ = 1.0.
- El link del LiDAR (`lidar_1`) está modelado, pero el sensor está desactivado: este grupo no implementa SLAM.
- El plugin de Gazebo es `gz-sim-ackermann-steering-system`: recibe `/cmd_vel` y publica `/odom` y `/tf`.

### Modelo cinemático (bicicleta, referencia en el eje trasero)

```
ẋ = v·cosθ      ẏ = v·sinθ      θ̇ = (v/L)·tanδ
R = L / tanδ     R_min = L / tan δ_max
```

---

## 6. Controlador: Pure Pursuit

1. Lee la pose de `/odom` y la traslada al eje trasero (0.12 m detrás del origen del modelo).
2. Busca el punto de la trayectoria más cercano y, desde él, el punto objetivo a distancia **L_d**.
3. Calcula la curvatura **κ = 2·sin(α) / L_d**, con α el ángulo entre la orientación del robot y el punto objetivo, y la satura a la curvatura máxima del robot.
4. Publica `v` y `ω = v·κ` en `/cmd_vel`. El plugin de Gazebo los convierte en los ángulos de dirección de cada rueda.
5. Al llegar al final, publica velocidad cero y guarda los resultados.

Trayectoria por defecto: lemniscata de Bernoulli con `a = 3 m` (6 m de largo, 15.7 m de recorrido, radio mínimo de curvatura 1.0 m).

---

## 7. Resultados

Trayectoria: infinito, `a = 3 m`, velocidad 0.4 m/s. Error transversal medido desde el eje trasero hasta el segmento más cercano de la trayectoria.

| L_d [m] | Completó | RMSE [cm] | Error máx. [cm] | Dirección saturada |
|---|---|---|---|---|
| 0.4 | No (inestable) | 58.3 | 155.5 | 80 % |
| 0.6 | Sí | **6.1** | 12.4 | 3.6 % |
| 1.0 | Sí | 8.7 | 14.4 | 1.8 % |

- Con L_d = 0.4 m el robot oscila y pierde la trayectoria.
- Con L_d = 0.6 m es la más precisa.
- Con L_d = 1.0 m es la más suave, pero se abre más en las curvas.

El valor RMSE que imprime el nodo (0.064 m con L_d = 0.6 m) es algo mayor que el de la tabla, porque mide hasta el punto más cercano de la trayectoria y no hasta el segmento.


### Datos

Cada corrida guarda en `~/ackermann_results/`:

- `run_*.csv`: `t, x, y, yaw, v_cmd, w_cmd, delta_est, target_x, target_y, cte`
- `path_*.csv`: trayectoria deseada (`x, y`)


---

## 8. Licencia

MIT.
