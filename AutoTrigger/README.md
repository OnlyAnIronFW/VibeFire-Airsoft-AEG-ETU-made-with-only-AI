# AutoTrigger — Computer Vision Auto-Aim System for Nerf Toys & AEG

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-%3E%3D3.16-brightgreen.svg)](https://cmake.org/)
[![Platform](https://img.shields.io/badge/Platform-RK3566%20%7C%20x86__64-lightgrey.svg)]()

**AutoTrigger** transforms your Nerf or AEG toy gun into a smart-scope system with real-time AI target detection, laser rangefinding, ballistic trajectory compensation, and an auto-trigger mechanism. Think of it as giving your foam-dart blaster the targeting capabilities of a modern FPS game.

> :warning: **This project is for recreational use only** — Nerf toys, gel blasters, and airsoft in legal venues. Verify local regulations before use. The system is designed for foam/water-gel projectiles at low velocities; using it with live firearms or in jurisdictions where electronic aiming aids are prohibited is your own liability.

---

## Table of Contents

- [How It Works](#how-it-works)
- [Key Specifications](#key-specifications)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
  - [Required Components (BOM)](#required-components-bom)
  - [Wiring Diagram](#wiring-diagram)
- [Software Stack](#software-stack)
  - [Dependencies](#dependencies)
  - [Project Structure](#project-structure)
- [Building from Source](#building-from-source)
  - [x86_64 Development Build (Mock Mode)](#x86_64-development-build-mock-mode)
  - [ARM64 Production Build (RK3566)](#arm64-production-build-rk3566)
  - [Cross-Compilation and Deployment](#cross-compilation-and-deployment)
- [Usage](#usage)
  - [Command-Line Options](#command-line-options)
  - [Calibration](#calibration)
  - [Systemd Autostart](#systemd-autostart)
- [Testing](#testing)
- [Roadmap](#roadmap)
- [License](#license)

---

## How It Works

Each frame (~33 ms at 30 FPS), the following pipeline executes:

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Camera      │────▶│  YOLOv5n NPU │────▶│  Kalman      │
│  (640x640)   │     │  Detection   │     │  Tracker     │
└──────────────┘     └──────────────┘     └──────┬───────┘
                                                 │
                  ┌──────────────────────────────┘
                  ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Laser ToF   │────▶│  Ballistic   │────▶│  Trigger     │
│  Ranging     │     │  Solver      │     │  Logic       │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                            │                    │
                            ▼                    ▼
                     ┌──────────────┐     ┌──────────────┐
                     │  OLED Display│     │  GPIO Output │
                     │  (240x240)   │     │  (Shot Fire) │
                     └──────────────┘     └──────────────┘
                            │
                 ┌──────────▼──────────┐
                 │  Safety Monitor     │
                 │  (Thermal + WDT +   │
                 │   Heartbeat)        │
                 └─────────────────────┘
```

1. **YOLO Detection** — A YOLOv5n model running on the RK3566 NPU detects persons in the 640x640 camera frame at 30 FPS.
2. **Kalman Tracking** — An 8-DOF Kalman filter tracks the target's position, velocity, and acceleration across frames, handling occlusions with coast prediction.
3. **Ranging** — Distance is obtained via a JRT TOF01 laser rangefinder (UART), fused with monocular fallback from bounding-box height for redundancy.
4. **Ballistics** — A precomputed drop table (bilinear interpolation over 801 ranges x 3 muzzle velocities) compensates for foam-dart drag and gravity, outputting vertical offset and time-of-flight.
5. **Aimpoint Display** — The compensated aimpoint (drop + lead + bore offset) is rendered on a 240x240 round OLED screen. The user's task: align the fixed crosshair with the moving green dot.
6. **Auto-Trigger** — When the aimpoint aligns within 8 pixels of the crosshair for 3 consecutive frames (consensus check), a GPIO signal energizes the blaster's trigger solenoid.
7. **Safety Layer** — A hardware-independent safety monitor enforces startup self-checks, CPU thermal throttling (85C→80C hysteresis), watchdog timer, and heartbeat LED. The trigger output is forcibly LOW during any fault condition.

---

## Key Specifications

| Parameter | Value |
|-----------|-------|
| Effective engagement range | 40-60 m |
| Person detection range | >= 40 m |
| Detection framerate | 30 FPS |
| End-to-end latency | ~38-52 ms (2-frame pipeline) |
| System power | ~5 W |
| Battery life | ~10 hours (10,000 mAh power bank) |
| Total weight | ~180 g |
| Unit cost (BOM) | ~755-785 RMB ($105-110 USD) |
| Target platform | Radxa Zero 3E (RK3566, Cortex-A55 x4 + NPU) |
| Dev platform | x86_64 (mock mode, no hardware required) |

---

## System Architecture

### Module Dependency Graph

```
main.cpp
  ├── Pipeline (orchestrator)
  │     ├── YOLOInfer (abstract) -> YOLOMock / [YOLOReal - RKNN, v1.1]
  │     ├── KalmanFilter (8-DOF, Eigen3)
  │     ├── Ranging (JRT TOF01 UART + monocular fallback)
  │     ├── Ballistics (precomputed drop table, bilinear interpolation)
  │     ├── Trigger (GPIO via libgpiod, 3-frame consensus)
  │     └── IDisplay (abstract) -> DisplayMock / [DisplayReal - SPI, v1.1]
  ├── Safety (hardware monitor: thermal, watchdog, heartbeat)
  └── Calibration (camera intrinsic, boresight, ballistics)
```

### Mock Mode Architecture

On x86_64 (development), `MOCK_MODE` is automatically defined. All hardware-dependent classes are replaced with mock implementations:

| Real Class | Mock Replacement | Behavior |
|------------|-----------------|----------|
| YOLOReal (RKNN) | `YOLOMock` | Returns pre-seeded detection boxes |
| DisplayReal (SPI OLED) | `DisplayMock` | Renders to a 240x240 in-memory framebuffer |
| Trigger (libgpiod) | `Trigger` (mock) | `is_held()` returns `false`, `fire()` is no-op |
| Safety hardware I/O | `SafetyMock` | Injectable sensor values, synthetic clock |

This allows full integration testing and pipeline development on any x86 machine without physical hardware.

---

## Hardware

### Required Components (BOM)

| # | Component | Model | Qty | Approx. Cost |
|---|-----------|-------|:---:|:------------:|
| 1 | SBC | Radxa Zero 3E 1GB (RK3566) | 1 | 250 RMB |
| 2 | Camera | IMX219 62-degree (non-fisheye!), MIPI CSI | 1 | 100 RMB |
| 3 | Laser ToF | JRT TOF01 (100 m UART) | 1 | 220 RMB |
| 4 | Display | 1.3" Round OLED 240x240 SPI | 1 | 55 RMB |
| 5 | Power Bank | 10,000 mAh USB-C | 1 | 45 RMB |
| 6 | MicroSD | 32 GB Class 10 | 1 | 25 RMB |
| 7 | Pin Headers and Dupont Wires | 2.54 mm (M+F) | various | 10 RMB |
| 8 | 3D Printed Enclosure | PETG filament | 1 set | 40 RMB |
| 9 | Nylon Standoffs | M2.5 x 6 mm | 4 | 5 RMB |
| 10 | Toggle Switch | SS-12D00 | 1 | 3 RMB |
| 11 | Heat Shrink Tubing | Assorted sizes | various | 2 RMB |
| | | | **Total** | **755 RMB** |

### Wiring Diagram

```
                 ┌──────────────────────────────────┐
                 │       Radxa Zero 3E (Back)        │
                 │                                  │
                 │  CSI (MIPI 15-pin FPC)           │
   IMX219────────┤  ┌──────────────────────────┐   │
   Camera        │  │                          │   │
                 │  │  GPIO 40-Pin Header       │   │
                 │  │  ┌───┬───┬───┬───┬───┐   │   │
                 │  │  │ 1 │ 2 │ 3 │...│ 40│   │   │
                 │  │  └─┬─┴─┬─┴─┬─┴─┬─┴─┬─┘   │   │
                 │  │    │   │   │   │   │      │   │
                 │  │    │   │   │   │   │      │   │
                 │  └────┼───┼───┼───┼───┼──────┘   │
                 │       │   │   │   │   │          │
                 │  Pin: │   │   │   │   │          │
                 │   8,10│   │   │   │   │          │
                 │  UART │   │   │   │   │          │
   JRT TOF01 ────┘       │   │   │   │          │
   Laser ToF             │   │   │   │          │
                         │   │   │   │          │
                 Pin: 19,23,18,22   │          │
                 SPI ───────────────┘          │
   OLED ─────────┘                   │          │
   Display                           │          │
                             Pin: 11,14         │
                             GPIO ──────────────┘
   Trigger ◄──────────────────┘
   Solenoid
```

| SBC Pin | Device | Signal |
|:-------:|--------|--------|
| **Camera** | | |
| CSI FPC | IMX219 | MIPI CSI (direct plug) |
| **Laser ToF (UART)** | | |
| Pin 8 (UART2_TX) | JRT TOF01 RX | Data receive |
| Pin 10 (UART2_RX) | JRT TOF01 TX | Data transmit |
| **OLED (SPI)** | | |
| Pin 19 (SPI1_MOSI) | OLED DIN | SPI data |
| Pin 23 (SPI1_SCLK) | OLED CLK | SPI clock |
| Pin 18 (GPIO) | OLED DC | Data/Command |
| Pin 22 (GPIO) | OLED RST | Reset |
| **Trigger** | | |
| Pin 11 (GPIO) | Solenoid IN | Fire signal |
| **Power** | | |
| USB-C | Power bank | 5 V input |

> **Note**: v1.0 currently uses `DisplayMock` and `YOLOMock` for development. Real SPI display and RKNN YOLO inference are planned for v1.1. The `safety.cpp` startup self-check targets VL53L1X (I2C) - using JRT TOF01 (UART) will report a ToF anomaly (system runs in degraded mode, trigger disabled). This will be fixed in v1.1.

---

## Software Stack

### Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| C++ Standard | 17 | Language features |
| CMake | >= 3.16 | Build system |
| OpenCV | 4.x | Image processing, camera calibration |
| Eigen3 | 3.4.0 | Linear algebra (Kalman filter) |
| SDL2 | 2.x | Display rendering (mock/testing) |
| GoogleTest | 1.14.0 | Unit testing framework |
| RKNN SDK | 2.x | NPU inference runtime (ARM64 only) |
| libgpiod | >= 1.6 | GPIO control (ARM64 only) |

### Project Structure

```
AutoTrigger/
├── CMakeLists.txt              # Top-level build config
├── LICENSE                     # GPLv3
├── README.md
│
├── include/autotrigger/        # Public headers
│   ├── ballistics.h            #   Drop-table engine (801x3, bilinear interpolation)
│   ├── calibrate.h             #   Camera / boresight / ballistics calibration
│   ├── display.h               #   DisplayMock (240x240 SW framebuffer)
│   ├── kalman_filter.h         #   8-DOF Kalman (cx,cy,vx,vy,ax,ay,w,h)
│   ├── pipeline.h              #   Pipeline orchestrator (YOLO->Kalman->Ranging->Ballistics->Trigger)
│   ├── ranging.h               #   JRT TOF01 UART + monocular fallback fusion
│   ├── safety.h                #   Safety monitor (thermal + watchdog + heartbeat)
│   ├── trigger.h               #   GPIO trigger (3-frame consensus)
│   ├── yolo_infer.h            #   YOLOInfer interface + YOLOMock + NMS helpers
│   └── hal/                    #   Hardware abstraction layer
│       ├── icamera.h           #     ICamera interface
│       ├── idisplay.h          #     IDisplay interface
│       ├── iranging.h          #     IRanging interface
│       └── itrigger.h          #     ITrigger interface
│
├── src/                        # Implementation
│   ├── main.cpp                #   CLI entry point, main loop, signal handling
│   ├── ballistics.cpp          #   Drop-table loading, bilinear interp, analytic fallback
│   ├── calibrate.cpp           #   Camera, boresight, ballistics calibration procedures
│   ├── display.cpp             #   DisplayMock: crosshair + aimpoint on 240x240 framebuffer
│   ├── kalman_filter.cpp       #   Predict/update/coast cycle, adaptive R, Mahalanobis gating
│   ├── pipeline.cpp            #   Full pipeline: detect->track->range->ballistics->trigger->display
│   ├── ranging.cpp             #   UART frame parsing, confidence-weighted fusion
│   ├── safety.cpp              #   Startup POST, thermal monitor, watchdog, heartbeat LED
│   ├── trigger.cpp             #   GPIO fire control (mock on x86, libgpiod on ARM64)
│   └── yolo_infer.cpp          #   YOLOMock: deterministic test detection
│
├── tests/                      # Unit tests (GoogleTest)
│   ├── CMakeLists.txt
│   ├── test_ballistics.cpp     #   Drop-table loading, interpolation, analytic fallback
│   ├── test_display.cpp        #   Crosshair + aimpoint + round mask rendering
│   ├── test_kalman.cpp         #   Init, prediction, update, coast, lost detection
│   ├── test_pipeline.cpp       #   Full integration: YOLO->Kalman->Ballistics->Trigger
│   ├── test_ranging.cpp        #   UART frame parsing, fusion, monocular fallback
│   ├── test_safety.cpp         #   Startup POST, thermal hysteresis, watchdog, heartbeat
│   ├── test_sanity.cpp         #   Catch-all compilation sanity check
│   ├── test_trigger.cpp        #   3-frame consensus, aim alignment, safety interlock
│   └── test_yolo.cpp           #   NMS, IoU computation, mock detection injection
│
├── scripts/                    # Utility scripts
│   ├── generate_drop_table.py  #   Physics sim: RK2 integration, quadratic drag, 2 g dart
│   ├── deploy.sh               #   Cross-compile + SCP deploy to Radxa Zero 3E
│   └── setup_rk3566.sh         #   One-shot board setup (apt deps, rknpu modprobe)
│
├── cmake/
│   └── aarch64-linux-gnu.cmake #   Cross-compilation toolchain (RK3566 target)
│
├── tables/                     # Runtime data
│   └── drop_table.bin          #   801x3 float32 (range 0-80 m, v0 = 55/70/90 m/s)
│
├── models/                     # YOLO models (place .rknn files here)
│   └── .gitkeep
│
├── calib/                      # Calibration data
│   └── .gitkeep
│
├── logs/                       # Runtime logs
│
└── docs/
    └── ASSEMBLY_GUIDE.md       # Hardware assembly and usage tutorial (Chinese)
```

---

## Building from Source

### x86_64 Development Build (Mock Mode)

No physical hardware required. Mock mode is enabled automatically on non-ARM64 targets.

```bash
# Prerequisites (Ubuntu/Debian)
sudo apt install -y build-essential cmake git \
    libopencv-dev libsdl2-dev libeigen3-dev

# Clone
git clone https://github.com/yourname/autotrigger.git
cd autotrigger

# Generate ballistics drop table
python3 scripts/generate_drop_table.py

# Build (Release)
mkdir -p build_x86 && cd build_x86
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)

# Run tests
ctest --output-on-failure

# Run the application (mock mode)
./autotrigger --v0 70
```

### ARM64 Production Build (RK3566)

Perform on the Radxa Zero 3E board running Armbian/Debian.

```bash
# One-time board setup
chmod +x scripts/setup_rk3566.sh
./scripts/setup_rk3566.sh
sudo reboot

# After reboot
git clone https://github.com/yourname/autotrigger.git
cd autotrigger

# Generate drop table
python3 scripts/generate_drop_table.py

# Build (Release, ARM64 native)
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
make -j4

# Install
sudo make install

# Run
autotrigger --v0 70
```

### Cross-Compilation and Deployment

Build on x86 host, deploy to board via SCP:

```bash
# Prerequisite: aarch64 cross-compilation toolchain
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# One-command cross-build + deploy
./scripts/deploy.sh radxa /home/radxa/autotrigger
```

---

## Usage

### Command-Line Options

```bash
autotrigger [options]

Options:
  --model PATH    YOLO model path           (default: models/yolov5n.rknn)
  --table PATH    Drop table path           (default: tables/drop_table.bin)
  --uart DEV      Laser rangefinder UART    (default: /dev/ttyS1)
  --v0 MPS        Muzzle velocity (m/s)     (default: 70, range: 55-90)

Examples:
  autotrigger --v0 80                       # 80 m/s muzzle velocity
  autotrigger --uart /dev/ttyS2 --v0 65     # Alternative UART, lower velocity
```

### Calibration

Calibration must be performed whenever you change hardware configuration (new gun, re-mounted camera, different dart type). See `docs/ASSEMBLY_GUIDE.md` for detailed step-by-step instructions.

**Quick start:**

```cpp
// 1. Camera intrinsics (one-time)
autotrigger::CameraCalibration cam_calib;
autotrigger::Calibration::calibrate_camera("./chessboard_images/", cam_calib);
// -> Update FOCAL constant in pipeline.h

// 2. Boresight alignment (at exactly 15 m)
autotrigger::BoreCalibration bore_calib;
autotrigger::Calibration::calibrate_boresight(15.0f, pixel_offset, bore_calib);
// -> Update BORE_HEIGHT / BORE_REF_DIST in pipeline.h

// 3. Ballistics (3 distances, 10 shots each, measure average drop)
autotrigger::BallisticCalibration bal_calib;
autotrigger::Calibration::calibrate_ballistics(drop_m, range_m, bal_calib);
// -> Use bal_calib.muzzle_velocity as --v0 parameter
```

### Systemd Autostart

```bash
sudo tee /etc/systemd/system/autotrigger.service << 'EOF'
[Unit]
Description=AutoTrigger Auto-Aim System
After=network.target

[Service]
Type=simple
User=radxa
WorkingDirectory=/home/radxa/autotrigger
ExecStart=/usr/local/bin/autotrigger --v0 70
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable autotrigger
sudo systemctl start autotrigger
```

---

## Testing

The project uses GoogleTest with a comprehensive test suite covering every module in isolation and integration.

```bash
# Build with tests (default)
cd build_x86 && cmake .. -DBUILD_TESTING=ON

# Build and run all tests
cmake --build . && ctest --output-on-failure

# Run a specific test
./tests/test_pipeline
```

### Test Coverage

| Test File | Modules Covered | Focus |
|-----------|----------------|-------|
| `test_ballistics` | Ballistics | Drop-table I/O, bilinear interpolation, analytic fallback, boundary conditions |
| `test_display` | DisplayMock | Crosshair rendering, aimpoint rendering, round mask clipping |
| `test_kalman` | KalmanFilter | State init, predict/update cycle, coast tracking, convergence, lost detection |
| `test_pipeline` | Pipeline, YOLOMock | End-to-end: detection -> tracking -> ranging -> ballistics -> alignment check |
| `test_ranging` | Ranging, RangingMock | UART frame parsing, monocular ranging, confidence-weighted fusion |
| `test_safety` | Safety, SafetyMock | POST checks, thermal hysteresis, watchdog timeout, heartbeat toggling |
| `test_trigger` | Trigger, TriggerMock | 3-frame consensus, alignment gating, safety interlock |
| `test_yolo` | YOLOMock | NMS filtering, IoU computation, mock detection injection |
| `test_sanity` | - | Compilation sanity: all headers included |

### Mock Infrastructure

All hardware-dependent classes expose injectable test doubles:

```cpp
// Example: testing pipeline trigger logic
SafetyMock safety(/* camera */ nullptr, /* ranging */ nullptr, /* trigger */ nullptr);
safety.set_mock_tof_healthy(false);  // Simulate ToF failure
StartupResult result = safety.do_startup_check();
ASSERT_TRUE(result.can_proceed);
ASSERT_TRUE(result.degraded);        // Degraded mode active
```

---

## Roadmap

### v1.0 - Current

- [x] YOLOv5n interface (abstract) + YOLOMock for x86 development
- [x] 8-DOF Kalman filter (position, velocity, acceleration, bbox size)
- [x] JRT TOF01 UART laser ranging with monocular fallback
- [x] Precomputed drop table (801 ranges x 3 velocities, quadratic drag physics)
- [x] Pipeline orchestrator (YOLO -> Kalman -> Ranging -> Ballistics -> Trigger -> Display)
- [x] Safety monitor (thermal hysteresis, watchdog timer, heartbeat LED)
- [x] 3-frame trigger consensus (anti-bounce debouncing)
- [x] DisplayMock (240x240 software framebuffer for development)
- [x] Full GoogleTest suite (9 test files, all modules covered)
- [x] CMake build system with dual-target support (x86 mock / ARM64 real)
- [x] Cross-compilation toolchain + deployment script

### v1.1 - Planned

- [ ] Real YOLOv5n RKNN inference on RK3566 NPU
- [ ] Real OLED SPI display driver (replace DisplayMock)
- [ ] UART ToF self-check support (JRT TOF01 startup POST)
- [ ] Camera calibration CLI flag
- [ ] YOLOv5n to RKNN model conversion script (`scripts/convert.py`)
- [ ] Enhanced bounding-box temporal filtering (reduce false positives)
- [ ] Detailed runtime profiling and latency telemetry

### Future Ideas

- [ ] Multi-target priority selection (closest / most threatening)
- [ ] IR illuminator integration for night operations
- [ ] Bluetooth companion app (battery status, calibration, diagnostics)
- [ ] Recoil compensation via IMU sensor fusion
- [ ] On-device training support via transfer learning

---

## License

AutoTrigger is free software: you can redistribute it and/or modify it under the terms of the [GNU General Public License v3.0](LICENSE) or (at your option) any later version.

```
AutoTrigger - Computer Vision Auto-Aim for Nerf Toys and AEG
Copyright (C) 2025

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
```

---

*AutoTrigger v1.0 — Built for the Nerf battlefield. May your darts fly true.*

---

# AutoTrigger — 海绵软弹/水弹 计算机视觉自动瞄准系统

**AutoTrigger** 将你的 Nerf/AEG 玩具枪变成智能瞄准系统，具备实时 AI 目标检测、激光测距、弹道补偿和自动激发功能。让真人 CS 体验 FPS 游戏的自瞄乐趣。

> :warning: **仅限娱乐用途** — 海绵软弹、水弹、合法场地的 Airsoft。使用前请核实当地法规。本系统专为低速泡沫/水凝胶弹丸设计；严禁用于真枪或在禁止电子瞄准辅助的场合使用。

---

## 工作原理

每帧（~33 ms，30 FPS）执行以下流水线：

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  IMX219 摄像头│────▶│  YOLOv5n NPU │────▶│  Kalman 滤波 │
│  (640x640)   │     │  人物检测     │     │  目标追踪     │
└──────────────┘     └──────────────┘     └──────┬───────┘
                                                 │
                  ┌──────────────────────────────┘
                  ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  JRT TOF01   │────▶│  弹道解算    │────▶│  触发逻辑     │
│  激光测距     │     │              │     │              │
└──────────────┘     └──────┬───────┘     └──────┬───────┘
                            │                    │
                            ▼                    ▼
                     ┌──────────────┐     ┌──────────────┐
                     │  OLED 显示器  │     │  GPIO 激发   │
                     │  (240x240)   │     │  信号输出     │
                     └──────────────┘     └──────────────┘
                            │
                 ┌──────────▼──────────┐
                 │  安全监控           │
                 │  (温度 + 看门狗 +   │
                 │   心跳灯)           │
                 └─────────────────────┘
```

1. **YOLO 人物检测** — YOLOv5n 模型在 RK3566 NPU 上运行，在 640x640 画面中实时检测人物，30 FPS。
2. **Kalman 目标追踪** — 8 自由度卡尔曼滤波器追踪目标位置、速度和加速度，支持短时遮挡的惯性预测。
3. **激光测距** — JRT TOF01 激光测距仪（UART 串口）获取距离，融合单目 bounding-box 高度估算作为冗余。
4. **弹道补偿** — 预计算弹道表（801 距离 x 3 初速，双线性插值）补偿海绵软弹的空气阻力和重力下坠。
5. **瞄准点显示** — 补偿后的瞄准点（下坠 + 提前量 + 镜桥偏移）显示在 240x240 圆形 OLED 上。操作要领：移动枪口使十字准星对准绿色瞄点。
6. **自动激发** — 瞄准点与准星在 8 像素内持续对齐 3 帧（共识验证），GPIO 输出高电平驱动电磁铁激发。
7. **安全层** — 开机自检、CPU 过热保护（85°C 触发 → 80°C 恢复，滞回）、看门狗定时器、心跳指示灯。任何故障状态下强制拉低触发信号。

---

## 核心参数

| 参数 | 数值 |
|------|------|
| 有效交战距离 | 40-60 m |
| 人物检测距离 | >= 40 m |
| 检测帧率 | 30 FPS |
| 端到端延迟 | ~38-52 ms（2 帧流水线） |
| 系统功耗 | ~5 W |
| 续航时间 | ~10 小时（10000 mAh 充电宝） |
| 整机重量 | ~180 g |
| 总成本（BOM） | ¥755-785 |
| 目标平台 | Radxa Zero 3E（RK3566, Cortex-A55 x4 + NPU） |
| 开发平台 | x86_64（Mock 模式，无需硬件） |

---

## 系统架构

```
main.cpp
  ├── Pipeline（流水线调度器）
  │     ├── YOLOInfer（抽象接口）→ YOLOMock / [YOLOReal — RKNN, v1.1]
  │     ├── KalmanFilter（8-DOF, 基于 Eigen3）
  │     ├── Ranging（JRT TOF01 UART + 单目测距融合）
  │     ├── Ballistics（预计算弹道表，双线性插值）
  │     ├── Trigger（GPIO 通过 libgpiod，3 帧共识）
  │     └── IDisplay（抽象接口）→ DisplayMock / [DisplayReal — SPI, v1.1]
  ├── Safety（硬件监控：温度、看门狗、心跳）
  └── Calibration（摄像头内参、瞄具归零、弹道校准）
```

### Mock 模式

在 x86_64 开发环境下，`MOCK_MODE` 自动启用，所有硬件相关类替换为 Mock 实现：

| 真实类 | Mock 替代 | 行为 |
|--------|----------|------|
| YOLOReal（RKNN） | `YOLOMock` | 返回预设检测框 |
| DisplayReal（SPI OLED） | `DisplayMock` | 渲染到 240x240 内存帧缓冲 |
| Trigger（libgpiod） | `Trigger`（mock） | `is_held()` 返回 `false`，`fire()` 空操作 |
| Safety 硬件 I/O | `SafetyMock` | 可注入传感器值、合成时钟 |

无需任何物理硬件即可在 PC 上进行完整的集成测试和流水线开发。

---

## 硬件清单（BOM）

| # | 组件 | 型号 | 数量 | 约价 |
|---|------|------|:---:|-----:|
| 1 | 主板 | Radxa Zero 3E 1GB（RK3566） | 1 | ¥250 |
| 2 | 摄像头 | IMX219 62°（非鱼眼！），MIPI CSI | 1 | ¥100 |
| 3 | 激光测距 | JRT TOF01（100m UART） | 1 | ¥220 |
| 4 | 显示屏 | 1.3寸圆形 OLED 240x240 SPI | 1 | ¥55 |
| 5 | 充电宝 | 10000 mAh USB-C | 1 | ¥45 |
| 6 | TF 卡 | 32GB Class 10 | 1 | ¥25 |
| 7 | 排针+杜邦线 | 2.54mm 公母各一包 | 若干 | ¥10 |
| 8 | 3D 打印外壳 | PETG 材质 | 1 套 | ¥40 |
| 9 | 尼龙螺柱 | M2.5 x 6mm | 4 | ¥5 |
| 10 | 拨动开关 | SS-12D00 | 1 | ¥3 |
| 11 | 热缩管 | 各规格 | 若干 | ¥2 |
| | | | **总计** | **¥755** |

> **注意**：v1.0 当前使用 `DisplayMock` 和 `YOLOMock` 开发。真实 SPI 屏幕驱动和 RKNN YOLO 推理计划在 v1.1 实现。`safety.cpp` 的开机自检针对 VL53L1X（I2C）— 使用 JRT TOF01（UART）会报告 ToF 异常（系统以降级模式运行，禁用激发）。此问题将在 v1.1 修复。

### 接线表

| 主板引脚 | 设备 | 信号 |
|:-------:|------|------|
| **摄像头** | | |
| CSI FPC | IMX219 | MIPI CSI（直接插入） |
| **激光测距（UART）** | | |
| Pin 8（UART2_TX） | JRT TOF01 RX | 数据接收 |
| Pin 10（UART2_RX） | JRT TOF01 TX | 数据发送 |
| **OLED 屏幕（SPI）** | | |
| Pin 19（SPI1_MOSI） | OLED DIN | SPI 数据 |
| Pin 23（SPI1_SCLK） | OLED CLK | SPI 时钟 |
| Pin 18（GPIO） | OLED DC | 数据/命令 |
| Pin 22（GPIO） | OLED RST | 复位 |
| **激发信号** | | |
| Pin 11（GPIO） | 电磁铁 IN | 激发信号 |
| **供电** | | |
| USB-C | 充电宝 | 5V 输入 |

---

## 编译构建

### x86_64 开发构建（Mock 模式）

无需硬件，非 ARM64 平台自动启用 Mock 模式。

```bash
# 安装依赖（Ubuntu/Debian）
sudo apt install -y build-essential cmake git \
    libopencv-dev libsdl2-dev libeigen3-dev

# 克隆仓库
git clone https://github.com/OnlyAnIronFW/AutoTriggerForAEG-Nerf.git
cd AutoTriggerForAEG-Nerf

# 生成弹道表
python3 scripts/generate_drop_table.py

# 编译（Release）
mkdir -p build_x86 && cd build_x86
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)

# 运行测试
ctest --output-on-failure

# 运行程序（Mock 模式）
./autotrigger --v0 70
```

### ARM64 生产构建（RK3566）

在 Radxa Zero 3E 主板上执行（需 Armbian/Debian 系统）。

```bash
# 一次性主板初始化
chmod +x scripts/setup_rk3566.sh
./scripts/setup_rk3566.sh
sudo reboot

# 重启后
git clone https://github.com/OnlyAnIronFW/AutoTriggerForAEG-Nerf.git
cd AutoTriggerForAEG-Nerf

# 生成弹道表
python3 scripts/generate_drop_table.py

# 编译（Release, ARM64 原生）
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
make -j4

# 安装
sudo make install

# 运行
autotrigger --v0 70
```

### 交叉编译 + 部署

在 x86 宿主机上交叉编译，通过 SCP 部署到主板：

```bash
# 安装交叉编译工具链
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 一键交叉编译 + 部署
./scripts/deploy.sh radxa /home/radxa/autotrigger
```

---

## 使用说明

### 命令行参数

```bash
autotrigger [选项]

选项：
  --model PATH    YOLO 模型路径         （默认：models/yolov5n.rknn）
  --table PATH    弹道表路径            （默认：tables/drop_table.bin）
  --uart DEV      激光测距串口设备       （默认：/dev/ttyS1）
  --v0 MPS        初速 m/s             （默认：70，范围：55-90）

示例：
  autotrigger --v0 80                    # 80 m/s 初速
  autotrigger --uart /dev/ttyS2 --v0 65  # 其他串口，较低初速
```

### 校准流程

每次更换硬件配置（新枪、重新安装摄像头、更换弹种）后必须重新校准。详细步骤见 `docs/ASSEMBLY_GUIDE.md`。

```cpp
// 1. 摄像头内参校准（一次性）
autotrigger::CameraCalibration cam_calib;
autotrigger::Calibration::calibrate_camera("./棋盘格图片/", cam_calib);
// → 将得到的 fx, fy, cx, cy 填入 pipeline.h 中的 FOCAL 常量

// 2. 瞄具归零校准（在恰好 15 米处）
autotrigger::BoreCalibration bore_calib;
autotrigger::Calibration::calibrate_boresight(15.0f, 像素偏移, bore_calib);
// → 更新 pipeline.h 中的 BORE_HEIGHT / BORE_REF_DIST

// 3. 弹道校准（3 个距离，各打 10 发，测量平均下落）
autotrigger::BallisticCalibration bal_calib;
autotrigger::Calibration::calibrate_ballistics(下落米数, 距离米数, bal_calib);
// → 将 bal_calib.muzzle_velocity 作为 --v0 参数
```

### 开机自启

```bash
sudo tee /etc/systemd/system/autotrigger.service << 'EOF'
[Unit]
Description=AutoTrigger 自动瞄准系统
After=network.target

[Service]
Type=simple
User=radxa
WorkingDirectory=/home/radxa/autotrigger
ExecStart=/usr/local/bin/autotrigger --v0 70
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable autotrigger
sudo systemctl start autotrigger
```

---

## 测试

使用 GoogleTest 框架，覆盖所有模块的单元测试和集成测试。

```bash
# 带测试编译（默认）
cd build_x86 && cmake .. -DBUILD_TESTING=ON

# 编译并运行全部测试
cmake --build . && ctest --output-on-failure

# 运行单个测试
./tests/test_pipeline
```

### 测试覆盖

| 测试文件 | 覆盖模块 | 测试重点 |
|----------|---------|---------|
| `test_ballistics` | Ballistics | 弹道表加载、双线性插值、解析回退、边界条件 |
| `test_display` | DisplayMock | 准星渲染、瞄准点渲染、圆形遮罩裁剪 |
| `test_kalman` | KalmanFilter | 状态初始化、预测/更新循环、惯性滑行、目标丢失判定 |
| `test_pipeline` | Pipeline, YOLOMock | 端到端：检测→追踪→测距→弹道→对准检查 |
| `test_ranging` | Ranging, RangingMock | UART 帧解析、单目测距、置信度加权融合 |
| `test_safety` | Safety, SafetyMock | 开机自检、温度滞回、看门狗超时、心跳切换 |
| `test_trigger` | Trigger, TriggerMock | 3 帧共识、对准门控、安全联锁 |
| `test_yolo` | YOLOMock | NMS 滤波、IoU 计算、Mock 检测注入 |
| `test_sanity` | — | 编译完整性检查 |

---

## 路线图

### v1.0 — 当前

- [x] YOLOv5n 抽象接口 + YOLOMock（x86 开发）
- [x] 8 自由度卡尔曼滤波器（位置、速度、加速度、检测框尺寸）
- [x] JRT TOF01 UART 激光测距 + 单目测距融合
- [x] 预计算弹道表（801 距离 x 3 初速，二次阻力物理模型）
- [x] 流水线调度器（YOLO → Kalman → Ranging → Ballistics → Trigger → Display）
- [x] 安全监控（温度滞回、看门狗、心跳灯）
- [x] 3 帧触发共识（防抖动）
- [x] DisplayMock（240x240 软件帧缓冲）
- [x] 完整 GoogleTest 测试套件（9 个测试文件，全模块覆盖）
- [x] CMake 双目标构建系统（x86 Mock / ARM64 真实）
- [x] 交叉编译工具链 + 部署脚本

### v1.1 — 计划

- [ ] 真实 YOLOv5n RKNN 推理（RK3566 NPU）
- [ ] 真实 OLED SPI 屏幕驱动（替代 DisplayMock）
- [ ] UART ToF 开机自检支持（JRT TOF01）
- [ ] 摄像头校准命令行参数
- [ ] YOLOv5n → RKNN 模型转换脚本（`scripts/convert.py`）
- [ ] 增强检测框时序滤波（降低误检率）
- [ ] 详细运行时性能分析和延迟遥测

### 未来构想

- [ ] 多目标优先级选择（最近/最具威胁）
- [ ] IR 补光灯夜战支持
- [ ] 蓝牙配套 App（电量、校准、诊断）
- [ ] IMU 传感器融合后坐力补偿
- [ ] 端侧迁移学习支持

---

## 开源协议

AutoTrigger 为自由软件：你可以依据 [GNU 通用公共许可证 v3.0](LICENSE)（或任何更新版本）的条款重新分发和/或修改它。

```
AutoTrigger — 海绵软弹/水弹 计算机视觉自动瞄准系统
Copyright (C) 2025

本程序为自由软件；您可依据自由软件基金会所发表的 GNU 通用公共许可证
（版本 3 或任何更新版本）的条款，重新发布和/或修改它。

本程序是基于使用目的而加以发布，然而不负任何担保责任；
亦无对适售性或特定目的适用性所为的默示性担保。
详情请参照 GNU 通用公共许可证。
```

---

*AutoTrigger v1.0 — 为真人 CS 而生，让每一发都精准命中。*
