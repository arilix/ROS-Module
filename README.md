# Robot Operating System (ROS) — Modul Pembelajaran

Selamat datang di modul pembelajaran **Robot Operating System (ROS)**! Di sini kita akan mempelajari bagaimana robot mengelola data dari berbagai sensor, mengolahnya, dan menghasilkan kontrol yang baik. ROS memungkinkan robot untuk mengerjakan task secara otonom — bisa dibilang, ROS adalah **otak** dari robot.

## Apa itu ROS?

**ROS (Robot Operating System)** adalah framework open-source yang menjadi fondasi utama dalam pengembangan perangkat lunak robot modern. Meskipun namanya "Operating System", ROS bukanlah OS seperti Linux atau Windows, melainkan **middleware** yang menyediakan infrastruktur komunikasi, tools, dan library untuk robotika.

### ROS 1 vs ROS 2

| Aspek | ROS 1 | ROS 2 |
|-------|-------|-------|
| Arsitektur | Butuh `roscore` (master node) | Decentralized (DDS-based) |
| Real-time | Tidak mendukung | Mendukung real-time |
| Platform | Linux only (resmi) | Linux, Windows, macOS |
| Security | Tidak ada built-in | Mendukung DDS Security |
| Multi-robot | Sulit | Native support |
| Status | Maintenance only (EOL) | Aktif dikembangkan |

### Distribusi ROS yang Populer

| Distribusi | Versi | OS Utama | Status |
|------------|-------|----------|--------|
| **ROS 1 Noetic** | ROS 1 | Ubuntu 20.04 | EOL Mei 2025 |
| **ROS 2 Foxy** | ROS 2 | Ubuntu 20.04 | EOL Mei 2023 |
| **ROS 2 Humble** | ROS 2 | Ubuntu 22.04 | LTS, EOL Mei 2027 |
| **ROS 2 Jazzy** | ROS 2 | Ubuntu 24.04 | LTS, EOL Mei 2029 |

> **Modul ini menggunakan ROS 2 Jazzy** sebagai distro utama untuk praktik, namun konsep yang dibahas berlaku untuk semua distribusi ROS 2 (dan sebagian besar juga relevan untuk ROS 1).

### Kegunaan ROS dalam Robot

ROS memudahkan integrasi berbagai sensor, aktuator, dan algoritma sehingga robot dapat:

- Mengelola dan memproses data dari banyak sensor secara real-time
- Mengontrol motor dan aktuator dengan presisi
- Berkomunikasi antar bagian robot secara modular (node-based)
- Menyusun sistem robotik yang scalable, reusable, dan mudah dikembangkan
- Mendukung komunikasi antar robot (multi-robot systems)
- Menyediakan tools untuk simulasi, visualisasi, dan debugging

Dengan ROS, robot dapat menjalankan tugas-tugas otonom seperti navigasi, manipulasi objek, pengenalan lingkungan, hingga kolaborasi antar robot. ROS mendukung berbagai platform dan bahasa pemrograman (Python, C++), sehingga sangat fleksibel untuk riset maupun industri.

## Daftar Isi Modul

1. [Pengenalan ROS](docs/01_pengenalan_ros2.md)
2. [Instalasi ROS 2](docs/02_instalasi.md)
3. [Sejarah & Distribusi ROS](docs/03_distribusi_ros.md)
4. [Perintah-Perintah Dasar ROS 2](docs/04_perintah_dasar.md)
5. [Membuat Workspace ROS 2](docs/05_membuat_workspace.md)
6. [Task & Latihan](docs/06_suprizeee.md)
7. [Closing & QNA](docs/07_closing.md)

## Quick Start

```bash
# Source ROS 2 environment (sesuaikan distro)
source /opt/ros/jazzy/setup.bash   # Jazzy (Ubuntu 24.04)
# source /opt/ros/humble/setup.bash # Humble (Ubuntu 22.04)

# Clone repository ini
cd ~/Documents/project
git clone <your-repo-url>
cd ros2_modules-main

# Buat workspace, build, dan source
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Apa yang Ada di Dalam Modul?

Mulai dari tutorial pertama dan ikuti secara berurutan. Setiap modul dilengkapi dengan:
- Penjelasan konsep
- Contoh kode (C++ dan Python)
- Latihan praktis
- Tips dan troubleshooting

## Requirements

- **Ubuntu 24.04** (untuk ROS 2 Jazzy) atau **Ubuntu 22.04** (untuk ROS 2 Humble)
- ROS 2 terinstal (Jazzy atau Humble)
- Dasar pemrograman Python atau C++
- Familiar dengan terminal Linux

---

*Happy learning!*
