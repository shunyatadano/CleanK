# CleanK 🚽🤖
*Re-imagining toilet hygiene with an autonomous quadruped cleaning robot.*

<div align="center">
  <img src="https://github.com/user-attachments/assets/348a2adb-43c8-42e4-980f-0514747ec650" width="420" alt="CleanK prototype">
</div>

[![CI](https://img.shields.io/badge/build-coming--soon-lightgrey?logo=githubactions)](#) 
[![Docs](https://img.shields.io/badge/docs-0%25-lightgrey?logo=readthedocs)](#) 
[![License](https://img.shields.io/badge/license-MIT-green)](#license)

---

## 🦾On going Demo
Waiting for your contribution!!
Proudly Desined by @sabamiso-rrsc 
https://x.com/sabamiso_RRSC
https://github.com/sabamiso-rrsc

https://github.com/user-attachments/assets/05408393-df8f-465e-b853-2cc754531bc4


![image 7](https://github.com/user-attachments/assets/416e029d-c5b0-4071-8188-8c5b091a95a0)

![image 6](https://github.com/user-attachments/assets/5153a75d-96bd-4ba9-adb9-8dbcfddd16e6)


## ✨ Why CleanK?

Maintaining public or commercial restrooms is expensive, repetitive, and—let’s be honest—unpleasant.  
CleanK turns that chore into a closed-loop, data-driven process:

| Layer | Goal | Key tech |
|-------|------|----------|
| **Machine Design** | Modular cleaning head + quadruped base | CAD / BOM (Fusion 360, Meviy sheet-metal & resin) |
| **Circuit Design** | Low-noise, IP-rated power & I/O | Custom carrier for off-the-shelf SBC |
| **Embedded FW** | Real-time safety & actuator control | STM32 + FreeRTOS |
| **Robot Control** | Locomotion, manipulation, autonomy | ROS 2 **Humble**, Navigation 2, MoveIt 2, Gazebo-Sim |
| **Data & AI** | Dirt classification / RL policy | CV model + RL fine-tuning on collected soiling dataset |

---

## 🗺 Roadmap

<div align="center">
  <img src="https://github.com/user-attachments/assets/5e59aa33-6540-440b-a1e4-caa9e8eae2fa" width="620" alt="project roadmap">
</div>

| Milestone | Target | Status |
|-----------|--------|--------|
| **Prototype-0** (proof) | Apr 2025 | ✅ completed |
| CAD + BOM freeze | May 2025 | 🔄 in progress |
| Base arrival & E-board spin #1 | Jul 2025 | ⏳ pending |
| Sim stack (Nav2 + MoveIt 2) | Oct 2025 | |
| ROS2 bridge to embedded FW | Feb 2026 | |
| RL on real hardware | Apr 2026 | |

> *Want to help?* Scroll to **Contributing** 👇

---

## 🔧 Quick start (simulation only)

Until the physical hardware ships, you can play in Gazebo:

```bash
# 1. Clone
git clone https://github.com/deBroglieeeen/CleanK.git
cd CleanK

# 2. Install ROS 2 Humble & dependencies
sudo apt install -y python3-colcon-common-extensions ros-humble-desktop-full
rosdep install -r --from-paths src --ignore-src -y

# 3. Build
colcon build --symlink-install
source install/setup.bash

# 4. Launch sim
ros2 launch cleank_bringup gazebo.launch.py
Requires Ubuntu 22.04 + nvidia-driver 535+ for hardware-accelerated Gazebo rendering.

📂 Repository layout
bash
Copy
Edit
CleanK/
├── bringup/           # Robot launch files (ROS 2)
├── description/       # URDF + meshes for quadruped & cleaning head
├── docker/            # Dev-container & CI images
├── firmware/          # STM32 project (CubeIDE) – coming soon
├── hardware/          # STEP, STL, DXF exports – coming soon
├── scripts/           # One-shot dev utilities
└── tests/             # Pytest & Gazebo regression tests
(Folders marked coming soon will appear once the first CAD drop and PCB spin are published.)

🖥️ Development environment

Tool	Version	Notes
Ubuntu	22.04 (Jammy)	Primary dev OS
ROS 2	Humble Hawksbill	Foxy & Iron Irwin CI builds planned
Gazebo	Fortress	Classic support until Ignition migration
MoveIt	2.5+	Manipulation planning
STM32CubeIDE	1.15	Firmware
Fusion 360	2.0.*	Mechanical design
A reproducible Docker dev-container lives in docker/.

🧪 Testing
bash
Copy
Edit
colcon test --event-handlers console_direct+
# Hardware-in-the-loop tests (once robot arrives)
ros2 launch cleank_bringup hw_test.launch.py test:=wiper_force
CI will run unit tests + Linters (ament_lint, clang-tidy).
HIL jobs trigger on hardware/* tagged PRs.

🙌 Contributing
Fork the repo & create a feature branch.

Follow CONTRIBUTING.md (code-style, commit message convention).

Open a PR; one of the maintainers will review within 72 h.

Looking for help with:

Control theory (quadruped locomotion tuning)

ROS 2 behavior-tree authoring

Hardware integration & force-feedback cleaning head

Computer-vision dataset labelling

📜 License
Released under the MIT License—see LICENSE.
If you use CleanK in academic work, please cite this repository.

🗣 Contact / Support
Issues → GitHub issue tracker

Slack → invite link coming soon

Email → cleanK [at] example.com

Stay updated by starring ⭐ the repo and watching for releases.

<sub>© 2025 CleanK Project — Built with ❤️ and a mild obsession for sparkling tile floors.</sub>
