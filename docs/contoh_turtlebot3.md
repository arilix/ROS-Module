# Contoh: Simulasi TurtleBot3 (ROS 1 & ROS 2)

---

## ROS 1 Noetic

### Install TurtleBot3

```bash
sudo apt install ros-noetic-turtlebot3*
```

### Terminal 1 — Jalankan Gazebo World

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Terminal 2 — Jalankan RViz

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

### Terminal 3 — Teleop (Kontrol Keyboard)

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

> Jika teleop belum terinstall:
> ```bash
> sudo apt install ros-noetic-turtlebot3-teleop
> ```

### Kontrol Keyboard

| Tombol | Fungsi       |
|--------|--------------|
| W      | Maju         |
| X      | Mundur       |
| A      | Belok Kiri   |
| D      | Belok Kanan  |
| S      | Stop         |

---

## ROS 2 Foxy

### Install TurtleBot3

```bash
sudo apt install ros-foxy-turtlebot3*
```

### Set Environment

Tambahkan ke setiap terminal atau masukkan ke `~/.bashrc`:

```bash
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
```

### Terminal 1 — Jalankan Gazebo World

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 — Jalankan RViz2

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### Terminal 3 — Teleop (Kontrol Keyboard)

```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

> Jika teleop belum terinstall:
> ```bash
> sudo apt install ros-foxy-turtlebot3-teleop
> ```

### Kontrol Keyboard

| Tombol | Fungsi       |
|--------|--------------|
| W      | Maju         |
| X      | Mundur       |
| A      | Belok Kiri   |
| D      | Belok Kanan  |
| S      | Stop         |

---

## ROS 2 Humble

### Install TurtleBot3

```bash
sudo apt install ros-humble-turtlebot3*
```

### Set Environment

Tambahkan ke setiap terminal atau masukkan ke `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### Terminal 1 — Jalankan Gazebo World

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 — Jalankan RViz2

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### Terminal 3 — Teleop (Kontrol Keyboard)

```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

> Jika teleop belum terinstall:
> ```bash
> sudo apt install ros-humble-turtlebot3-teleop
> ```

### Kontrol Keyboard

| Tombol | Fungsi       |
|--------|--------------|
| W      | Maju         |
| X      | Mundur       |
| A      | Belok Kiri   |
| D      | Belok Kanan  |
| S      | Stop         |

---

## Perbedaan Utama

| Aspek             | ROS 1 Noetic          | ROS 2 Foxy / Humble            |
|-------------------|-----------------------|---------------------------------|
| Build system      | `catkin`              | `colcon`                        |
| Launch command    | `roslaunch`           | `ros2 launch`                   |
| Run command       | `rosrun`              | `ros2 run`                      |
| Launch file       | `.launch` (XML)       | `.launch.py` (Python)           |
| Visualisasi       | RViz                  | RViz2                           |
| Source setup      | `source /opt/ros/noetic/setup.bash` | `source /opt/ros/{foxy,humble}/setup.bash` |

---
---

# Mission Challenge: TurtleBot3 Autonomous Square Navigation

> Adaptasi dari misi drone MAVROS — dikonversi ke TurtleBot3 (ground robot).

## Objektif Utama

Mengembangkan script Python untuk melakukan **misi navigasi otonom** membentuk **pola persegi sempurna (2x2 meter)** menggunakan TurtleBot3 di Gazebo. Setelah kembali ke titik awal, robot berhenti secara otomatis.

## Logic Requirement: "Distance-Based Waypoint"

- Script **TIDAK BOLEH** hanya menggunakan `time.sleep()`
- Script harus terus menghitung jarak antara posisi robot saat ini (`/odom`) dengan koordinat target
- **Threshold**: Target dianggap tercapai jika jarak (Euclidean distance) sudah **< 0.3 meter**
- **Action**: Baru setelah threshold terpenuhi, script mengirimkan koordinat waypoint berikutnya

## Technical Constraints

| Constraint     | Detail                                                    |
|----------------|-----------------------------------------------------------|
| Frequency      | Script wajib mengirim `/cmd_vel` pada frekuensi **20Hz** |
| Safety         | Jika `/emergency_stop` bernilai `True`, script berhenti  |
| Pola           | Persegi 2x2 meter (4 waypoints)                          |
| Navigasi       | Rotate-then-move ke setiap waypoint                      |

## Waypoints (Relatif dari Posisi Awal)

```
WP1: (2.0, 0.0)  → maju 2m ke depan
WP2: (2.0, 2.0)  → belok kiri, maju 2m
WP3: (0.0, 2.0)  → belok kiri, maju 2m
WP4: (0.0, 0.0)  → kembali ke titik awal
```

```
  WP3 ─────────── WP2
   |                |
   |    PERSEGI     |
   |    2x2 m       |
   |                |
  WP4 ─────────── WP1
(start)
```

---

## Cara Pakai: ROS 1 Noetic

### 1. Build Workspace

```bash
cd workspace/ros1_ws
catkin_make
source devel/setup.bash
```

### 2. Terminal 1 — Jalankan Gazebo (Empty World)

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

### 3. Terminal 2 — Jalankan Square Mission

```bash
source devel/setup.bash
rosrun turtlebot3_square_mission square_mission.py
```

### Atau Pakai Launch File (Gazebo + Mission sekaligus)

```bash
source devel/setup.bash
roslaunch turtlebot3_square_mission square_mission.launch
```

### Emergency Stop (Opsional)

Dari terminal lain, publish emergency stop:
```bash
rostopic pub /emergency_stop std_msgs/Bool "data: true" --once
```

---

## Cara Pakai: ROS 2 Foxy / Humble

### 1. Build Workspace

```bash
cd workspace/ros2_ws
colcon build --packages-select turtlebot3_square_mission
source install/setup.bash
```

### 2. Terminal 1 — Jalankan Gazebo (Empty World)

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 3. Terminal 2 — Jalankan Square Mission

```bash
source install/setup.bash
ros2 run turtlebot3_square_mission square_mission
```

### Atau Pakai Launch File (Gazebo + Mission sekaligus)

```bash
source install/setup.bash
ros2 launch turtlebot3_square_mission square_mission.launch.py
```

### Custom Parameter

```bash
ros2 run turtlebot3_square_mission square_mission \
  --ros-args \
  -p side_length:=3.0 \
  -p threshold:=0.2 \
  -p linear_speed:=0.3
```

### Emergency Stop (Opsional)

Dari terminal lain, publish emergency stop:
```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

---

## Struktur File

```
workspace/
├── ros1_ws/
│   └── src/
│       └── turtlebot3_square_mission/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── launch/
│           │   └── square_mission.launch
│           └── scripts/
│               └── square_mission.py        ← Script utama (rospy)
│
└── ros2_ws/
    └── src/
        └── turtlebot3_square_mission/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── launch/
            │   └── square_mission.launch.py
            ├── resource/
            │   └── turtlebot3_square_mission
            └── turtlebot3_square_mission/
                ├── __init__.py
                └── square_mission.py        ← Script utama (rclpy)
```

## Perbedaan dengan Misi Drone (MAVROS)

| Aspek           | Drone (MAVROS)                     | TurtleBot3 (Ground Robot)              |
|-----------------|------------------------------------|----------------------------------------|
| Posisi          | `/mavros/local_position/pose`      | `/odom`                                |
| Perintah gerak  | `setpoint_position`                | `/cmd_vel` (Twist)                     |
| Mode            | GUIDED → LAND                      | Tidak ada mode (langsung stop)         |
| Dimensi         | 3D (x, y, z)                       | 2D (x, y) — robot di lantai           |
| Navigasi        | Set coordinate langsung            | Rotate + move (differential drive)     |
| Emergency       | Ganti mode ke STABILIZE            | Publish `/emergency_stop`              |
