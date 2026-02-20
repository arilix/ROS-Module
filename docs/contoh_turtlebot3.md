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

## Soal

Pada contoh di atas, TurtleBot3 dikendalikan secara **manual menggunakan teleop keyboard**. Sekarang tantangannya:

> **Buatlah script Python yang membuat TurtleBot3 bergerak secara OTONOM (tanpa teleop) membentuk pola persegi sempurna (2x2 meter) di Gazebo, lalu berhenti otomatis setelah kembali ke titik awal.**

### Yang Harus Kamu Buat:

1. **Buat package ROS** (pilih ROS 1 atau ROS 2) bernama `turtlebot3_square_mission`
2. **Buat script Python** yang menjadi pengganti teleop — robot jalan sendiri tanpa input keyboard
3. **Buat launch file** untuk menjalankan Gazebo + script mission sekaligus

---

## Ketentuan & Constraint

### Logic Requirement: "Distance-Based Waypoint"

- Script **TIDAK BOLEH** hanya menggunakan `time.sleep()` untuk berpindah waypoint
- Script harus terus **menghitung jarak** antara posisi robot saat ini (`/odom`) dengan koordinat target
- **Threshold**: Target dianggap tercapai jika jarak (Euclidean distance) sudah **< 0.3 meter**
- **Action**: Baru setelah threshold terpenuhi, script mengirimkan koordinat waypoint berikutnya

### Technical Constraints

| Constraint     | Detail                                                    |
|----------------|-----------------------------------------------------------|
| Frequency      | Script wajib mengirim `/cmd_vel` pada frekuensi **10Hz - 20Hz** |
| Safety         | Jika `/emergency_stop` bernilai `True`, script harus berhenti  |
| Pola           | Persegi 2x2 meter (4 waypoints)                          |
| Navigasi       | Rotate-then-move ke setiap waypoint                      |

### Waypoints yang Harus Dilalui (Relatif dari Posisi Awal)

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

## Hints & Panduan

### Topic yang Digunakan

| Topic              | Tipe Pesan              | Fungsi                                 |
|--------------------|-------------------------|----------------------------------------|
| `/odom`            | `nav_msgs/Odometry`     | Membaca posisi & orientasi robot       |
| `/cmd_vel`         | `geometry_msgs/Twist`   | Mengirim perintah gerak (linear & angular) |
| `/emergency_stop`  | `std_msgs/Bool`         | Safety — subscribe untuk berhenti      |

### Rumus yang Dibutuhkan

**Euclidean Distance** (untuk cek apakah sudah sampai waypoint):
```python
distance = sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
```

**Angle to Target** (untuk menentukan arah belok):
```python
angle = atan2(target_y - current_y, target_x - current_x)
```

**Quaternion ke Yaw** (orientasi dari `/odom` berbentuk quaternion, perlu dikonversi):
- ROS 1: gunakan `tf.transformations.euler_from_quaternion()`
- ROS 2: buat fungsi sendiri atau gunakan library `transforms3d`

### Algoritma yang Disarankan

```
untuk setiap waypoint:
  1. ROTATE: putar robot sampai menghadap target
     - hitung angle_diff = target_angle - current_yaw
     - publish angular.z sampai angle_diff < threshold
  2. MOVE: maju ke target
     - publish linear.x sambil koreksi angular.z
     - hitung euclidean_distance setiap loop
     - berhenti jika distance < 0.3 meter
  3. lanjut ke waypoint berikutnya
```

---

## Struktur Package yang Diharapkan

### Jika Pakai ROS 1 (Noetic)

```
ros1_ws/
└── src/
    └── turtlebot3_square_mission/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── square_mission.launch
        └── scripts/
            └── square_mission.py        ← Script utama (rospy)
```

**Cara test:**
```bash
# Terminal 1: Gazebo
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

# Terminal 2: Mission script
source devel/setup.bash
rosrun turtlebot3_square_mission square_mission.py
```

### Jika Pakai ROS 2 (Foxy / Humble)

```
ros2_ws/
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

**Cara test:**
```bash
# Terminal 1: Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Mission script
source install/setup.bash
ros2 run turtlebot3_square_mission square_mission
```

---

## Kriteria Penilaian

| Kriteria                                        | Bobot |
|-------------------------------------------------|-------|
| Robot berhasil membentuk pola persegi 2x2m      | 30%   |
| Menggunakan distance-based waypoint (bukan sleep) | 25%   |
| Frekuensi publish `/cmd_vel` sesuai (10-20Hz)   | 15%   |
| Ada logika emergency stop                        | 10%   |
| Robot kembali ke titik awal dan berhenti otomatis | 10%  |
| Kode bersih, ada komentar, dan bisa di-build     | 10%  |

---

## Referensi Perbandingan dengan Misi Drone (MAVROS)

> Soal ini diadaptasi dari mission challenge drone. Berikut perbedaannya:

| Aspek           | Drone (MAVROS)                     | TurtleBot3 (Ground Robot)              |
|-----------------|------------------------------------|----------------------------------------|
| Posisi          | `/mavros/local_position/pose`      | `/odom`                                |
| Perintah gerak  | `setpoint_position`                | `/cmd_vel` (Twist)                     |
| Mode            | GUIDED → LAND                      | Tidak ada mode (langsung stop)         |
| Dimensi         | 3D (x, y, z)                       | 2D (x, y) — robot di lantai           |
| Navigasi        | Set coordinate langsung            | Rotate + move (differential drive)     |
| Emergency       | Ganti mode ke STABILIZE            | Publish `/emergency_stop`              |

**Selamat mengerjakan!**
