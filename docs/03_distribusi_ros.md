# 3. Sejarah & Distribusi ROS

Modul ini membahas evolusi ROS dari ROS 1 hingga ROS 2, serta perbandingan antar distribusi agar kamu bisa memilih yang paling cocok untuk proyekmu.

---

## Apa itu Distribusi ROS?

Sama seperti Linux yang memiliki distribusi (Ubuntu, Fedora, dll), ROS juga memiliki **distribusi** (distribution/distro). Setiap distribusi:

- Memiliki **nama kode** unik (berurutan alfabet)
- Di-*build* dan diuji untuk **versi OS tertentu**
- Memiliki **masa support** yang terbatas (EOL = End of Life)
- Berisi kumpulan package yang sudah diverifikasi kompatibilitasnya

---

## ROS 1 — Generasi Pertama

### Timeline Distribusi ROS 1

| Distribusi | Tahun | Ubuntu | Status |
|------------|-------|--------|--------|
| Box Turtle | 2010 | 10.04 | EOL |
| Diamondback | 2011 | 10.04/11.04 | EOL |
| Fuerte | 2012 | 12.04 | EOL |
| Groovy | 2012 | 12.04/12.10 | EOL |
| Hydro | 2013 | 12.04/13.04 | EOL |
| Indigo | 2014 | 14.04 | EOL |
| Jade | 2015 | 14.04/15.04 | EOL |
| Kinetic | 2016 | 16.04 | EOL Apr 2021 |
| Lunar | 2017 | 16.04/17.04 | EOL |
| Melodic | 2018 | 18.04 | EOL Jun 2023 |
| **Noetic** | **2020** | **20.04** | **EOL Mei 2025** |

### ROS 1 Noetic Ninjemys

**Noetic** adalah distribusi **terakhir** dari ROS 1. Beberapa poin penting:

- **Python 3 only** — distribusi ROS 1 pertama yang sepenuhnya Python 3 (sebelumnya Python 2).
- **Ubuntu 20.04 (Focal Fossa)** sebagai target utama.
- Banyak digunakan di riset & industri yang belum migrasi ke ROS 2.
- Setelah EOL (Mei 2025), tidak ada lagi update keamanan.

### Arsitektur ROS 1

```
┌─────────────────────────────┐
│         roscore             │  ← Master node (WAJIB jalan)
│  ┌────────┐ ┌────────────┐ │
│  │ Master │ │ Param Srv  │ │
│  └────────┘ └────────────┘ │
└──────────┬──────────────────┘
           │ XMLRPC
    ┌──────┴──────┐
    ↓             ↓
┌────────┐   ┌────────┐
│ Node A │──→│ Node B │   ← TCPROS (peer-to-peer setelah discovery)
└────────┘   └────────┘
```

**Kelemahan:**
- `roscore` = single point of failure
- Tidak ada enkripsi/autentikasi
- Tidak real-time
- Hanya Linux

### Kapan Masih Perlu ROS 1?

- Menggunakan hardware/driver yang hanya tersedia untuk ROS 1
- Proyek legacy yang sudah berjalan dan belum bisa migrasi
- Paper/research yang membutuhkan package tertentu hanya ada di ROS 1
- Bisa menggunakan **ros1_bridge** untuk menghubungkan ROS 1 & ROS 2

---

## ROS 2 — Generasi Kedua

### Timeline Distribusi ROS 2

| Distribusi | Tahun | Ubuntu | Tipe | Status |
|------------|-------|--------|------|--------|
| Ardent | 2017 | 16.04 | Non-LTS | EOL |
| Bouncy | 2018 | 18.04 | Non-LTS | EOL |
| Crystal | 2018 | 18.04 | Non-LTS | EOL |
| Dashing | 2019 | 18.04 | LTS | EOL Mei 2021 |
| Eloquent | 2019 | 18.04 | Non-LTS | EOL |
| **Foxy** | **2020** | **20.04** | **LTS** | **EOL Mei 2023** |
| Galactic | 2021 | 20.04 | Non-LTS | EOL |
| **Humble** | **2022** | **22.04** | **LTS** | **EOL Mei 2027** |
| Iron | 2023 | 22.04 | Non-LTS | EOL Nov 2024 |
| **Jazzy** | **2024** | **24.04** | **LTS** | **EOL Mei 2029** |
| Rolling | terus | latest | Dev | Rolling release |

> **LTS (Long-Term Support)** = disupport selama ~5 tahun, cocok untuk produksi.
> **Non-LTS** = disupport ~1.5 tahun, untuk development & testing fitur baru.

### Arsitektur ROS 2

```
         ┌────────┐     DDS      ┌────────┐
         │ Node A │ ←──────────→ │ Node B │
         └────────┘   Discovery  └────────┘
              ↕         auto        ↕
         ┌────────┐              ┌────────┐
         │ Node C │ ←──────────→ │ Node D │
         └────────┘              └────────┘

         → Tidak ada master, semua peer-to-peer via DDS
```

**Keunggulan:**
- Decentralized — tidak ada single point of failure
- DDS middleware — standar industri untuk komunikasi real-time
- Multi-platform — Linux, Windows, macOS
- Security — DDS Security (enkripsi, autentikasi, access control)

---

## Perbandingan Detail Distribusi Populer

### ROS 1 Noetic vs ROS 2 Humble vs ROS 2 Jazzy

| Fitur | Noetic | Humble | Jazzy |
|-------|--------|--------|-------|
| **Generasi** | ROS 1 | ROS 2 | ROS 2 |
| **Rilis** | Mei 2020 | Mei 2022 | Mei 2024 |
| **EOL** | Mei 2025 | Mei 2027 | Mei 2029 |
| **Ubuntu** | 20.04 | 22.04 | 24.04 |
| **Python** | 3.8 | 3.10 | 3.12 |
| **Build tool** | catkin | colcon | colcon |
| **Middleware** | TCPROS/UDPROS | DDS | DDS |
| **Master node** | Ya (roscore) | Tidak | Tidak |
| **Simulator** | Gazebo Classic | Gazebo Fortress | Gazebo Harmonic |
| **CLI tool** | `rosrun`, `rostopic` | `ros2 run`, `ros2 topic` | `ros2 run`, `ros2 topic` |
| **Launch** | XML | Python / XML / YAML | Python / XML / YAML |

### Mana yang Harus Dipilih?

```
Mulai proyek baru?
├── Ya → Gunakan ROS 2
│       ├── Ubuntu 24.04? → Jazzy ✓ (recommended)
│       └── Ubuntu 22.04? → Humble ✓
│
└── Proyek existing ROS 1?
    ├── Bisa migrasi? → Migrasi ke ROS 2 (Humble/Jazzy)
    └── Belum bisa? → Tetap Noetic + ros1_bridge
```

---

## ROS 2 Foxy — Catatan Khusus

**Foxy Fitzroy** (Juni 2020) adalah distribusi LTS ROS 2 pertama yang banyak digunakan. Meskipun sudah EOL (Mei 2023), masih banyak tutorial dan proyek yang menggunakan Foxy. Jika kamu menemukan tutorial berbasis Foxy:

- Konsep dasarnya **sama** dengan Humble/Jazzy
- API di `rclpy` dan `rclcpp` **sangat mirip**
- Beberapa interface/package mungkin **berubah nama** atau **deprecated**
- Sebaiknya langsung gunakan Humble/Jazzy dan sesuaikan perintah

---

## Migrasi ROS 1 → ROS 2

### Perbedaan Perintah CLI

| Aksi | ROS 1 | ROS 2 |
|------|-------|-------|
| Build | `catkin_make` | `colcon build` |
| Run node | `rosrun pkg node` | `ros2 run pkg node` |
| List topics | `rostopic list` | `ros2 topic list` |
| Echo topic | `rostopic echo /topic` | `ros2 topic echo /topic` |
| List nodes | `rosnode list` | `ros2 node list` |
| Param get | `rosparam get /param` | `ros2 param get /node param` |
| Launch | `roslaunch pkg file.launch` | `ros2 launch pkg file.py` |
| Bag record | `rosbag record -a` | `ros2 bag record -a` |
| Source env | `source devel/setup.bash` | `source install/setup.bash` |

### ros1_bridge

Untuk proyek yang menggunakan campuran ROS 1 dan ROS 2, tersedia package **ros1_bridge** yang menghubungkan keduanya:

```bash
# Install ros1_bridge (membutuhkan ROS 1 dan ROS 2 terinstal)
sudo apt install ros-humble-ros1-bridge

# Jalankan bridge
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros1_bridge dynamic_bridge
```

> **Catatan**: ros1_bridge tersedia untuk Humble kemudian; untuk Jazzy, ketersediaan mungkin dikurangi karena ROS 1 sudah EOL.

---

## Rangkuman

- **ROS 1 Noetic** → Pilihan terakhir ROS 1, sudah/mendekati EOL. Gunakan hanya jika terpaksa.
- **ROS 2 Foxy** → EOL. Jangan digunakan untuk proyek baru.
- **ROS 2 Humble** → LTS, stabil, banyak package. Cocok untuk Ubuntu 22.04.
- **ROS 2 Jazzy** → LTS terbaru, fitur terlengkap. Cocok untuk Ubuntu 24.04.
- Jika memulai proyek baru, **selalu gunakan ROS 2** (Jazzy atau Humble).
- Migrasi dari ROS 1 ke ROS 2 bisa dilakukan bertahap dengan `ros1_bridge`.

## Next Steps

Lanjut ke [Perintah-Perintah Dasar ROS 2](04_perintah_dasar.md) untuk mempelajari CLI tools!

---

**Tips**: Selalu periksa versi distribusi yang kamu gunakan saat membaca tutorial online. Perintah dan package bisa berbeda antar distro!
