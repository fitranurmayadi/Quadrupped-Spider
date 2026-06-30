# Quadruped Spider Robot 🕷️

<p align="center">
  <img src="docs/spider_preview.png" alt="Quadruped Spider Robot" width="600"/>
</p>

A **4-legged quadruped spider robot** simulation and reinforcement learning environment built with **PyBullet** and **Gymnasium**. The 3D model is designed in **FreeCAD** and exported as STL meshes for physics simulation.

---

## 📋 Table of Contents

- [Features](#-features)
- [Project Structure](#-project-structure)
- [Coordinate System](#-coordinate-system)
- [Installation](#-installation)
- [Usage](#-usage)
- [Testing](#-testing)
- [Reinforcement Learning](#-reinforcement-learning)
- [Kinematics](#-kinematics)
- [Physics Parameters](#-physics-parameters)

---

## ✨ Features

- **Realistic 3D model** — Body, coxa, femur, and tibia meshes exported from FreeCAD
- **PyBullet physics** — Accurate rigid-body dynamics with joint damping, friction, and contact forces
- **Gymnasium environment** — Fully compatible with Stable-Baselines3 for RL training
- **Diagonal trot gait** — Forward, backward, lateral, and rotational locomotion
- **Attitude control** — Pitch/Roll body tilting via differential leg height
- **Interactive calibration** — Real-time slider-based joint tuning tool
- **Comprehensive tests** — Standing stability and locomotion direction tests

---

## 📁 Project Structure

```
Quadrupped-Spider/
│
├── assets/
│   ├── spider.urdf           # Robot URDF model (12 DOF, 4 legs × 3 joints)
│   ├── test_axis.urdf        # Coordinate axis visualization URDF
│   └── meshes/
│       ├── main_body.stl     # Robot chassis mesh
│       ├── coxa.stl          # Coxa (hip) segment mesh
│       ├── femur.stl         # Femur (upper leg) segment mesh
│       └── tibia.stl         # Tibia (lower leg) segment mesh
│
├── envs/
│   └── spider_env.py         # Gymnasium environment (SpiderEnv-v0)
│
├── testing/
│   ├── test_1_standing.py    # Test 1: Standing stability test
│   └── test_2_attitude.py    # Test 2: Attitude control & locomotion test
│
├── freecad/                  # FreeCAD source files (.FCStd)
├── models/                   # Saved RL model checkpoints
├── tensorboard_logs/         # Training logs for TensorBoard
│
├── train.py                  # PPO training script
├── display_robot.py          # Visualize robot in PyBullet GUI
├── interactive_calibration.py# Interactive joint calibration tool
├── basic_axis_check.py       # Coordinate axis alignment verification
└── requirements.txt
```

---

## 🧭 Coordinate System

This project uses **FreeCAD's default coordinate convention**, which differs from common robotics conventions:

| Direction | World Axis | Description |
|-----------|-----------|-------------|
| **Front** (Depan) | **−Y** | Robot faces negative Y |
| **Rear** (Belakang) | **+Y** | Back of robot |
| **Left** (Kiri) | **+X** | Robot's left side |
| **Right** (Kanan) | **−X** | Robot's right side |
| **Up** | **+Z** | Vertical up |

### Leg Positions

| Leg | Prefix | World Quadrant |
|-----|--------|---------------|
| Front-Left | `fl` | (+X, −Y) |
| Rear-Left  | `rl` | (+X, +Y) |
| Front-Right | `fr` | (−X, −Y) |
| Rear-Right | `rr` | (−X, +Y) |

### Joint Neutral Standing Angles

| Joint | fl | rl | fr | rr |
|-------|----|----|----|----|
| Coxa  | −0.5 rad | +0.5 rad | +0.5 rad | −0.5 rad |
| Femur | −1.2 rad | −1.2 rad | −1.2 rad | −1.2 rad |
| Tibia | +2.8 rad | +2.8 rad | +2.8 rad | +2.8 rad |

> **Note:** Right-side legs (fr, rr) have their coxa joint rotated 180° in URDF (`rpy="0 0 3.14159"`), so positive coxa angles splay outward for both sides.

---

## 🔧 Installation

### Prerequisites

- Python 3.9+
- pip

### Steps

```bash
git clone https://github.com/fitranurmayadi/Quadrupped-Spider.git
cd Quadrupped-Spider
pip install -r requirements.txt
```

### requirements.txt

```
gymnasium>=0.29.0
pybullet>=3.2.5
stable-baselines3>=2.0.0
numpy>=1.22.0
torch>=2.0.0
tensorboard>=2.10.0
```

---

## 🚀 Usage

### Visualize the Robot

```bash
python display_robot.py
```

Opens PyBullet GUI showing the robot in its default standing pose.

### Interactive Calibration Tool

```bash
python interactive_calibration.py
```

Opens PyBullet GUI with **debug sliders** to manually control all 12 joints, body position, and orientation. Press **`S`** to save current joint angles to `calibration_results.json`.

### Axis Alignment Check

```bash
python basic_axis_check.py
```

Displays a white reference box at origin and colored cylinders at each axis quadrant to visually verify the FreeCAD → PyBullet coordinate mapping.

---

## 🧪 Testing

### Test 1: Standing Stability

Verifies the robot can maintain a stable standing pose for an extended duration.

```bash
python testing/test_1_standing.py              # GUI mode
python testing/test_1_standing.py --headless   # Headless (faster)
python testing/test_1_standing.py --headless --duration 10.0
```

**Pass criteria:**
- Average body height > 0.02 m over last 2 seconds
- Body tilt (Roll, Pitch) < 0.3 rad
- ≥ 3 feet in ground contact

**Expected output:**
```
t=1.0s | Posisi (X,Y,Z): [0.000, 0.000, 0.028] m | Orientasi (R,P,Y): [0.000, 0.000, 0.000] rad | Kaki Menapak: 4/4
...
[STATUS] >>> TEST BERDIRI TEGAK: PASSED / LULUS <<<
```

### Test 2: Attitude Control & Locomotion

Tests body attitude control (Pitch/Roll) and directional locomotion (6 directions).

```bash
python testing/test_2_attitude.py              # GUI mode
python testing/test_2_attitude.py --headless   # Headless
python testing/test_2_attitude.py --headless --duration 3.0
```

**Tested motions:**

| Mode | Expected | Axis |
|------|---------|------|
| `maju` (forward) | dY < 0 | −Y |
| `mundur` (backward) | dY > 0 | +Y |
| `kiri` (left) | dX > 0 | +X |
| `kanan` (right) | dX < 0 | −X |
| `putar_kiri` (CCW) | dYaw > 0 | +Z rotation |
| `putar_kanan` (CW) | dYaw < 0 | −Z rotation |

---

## 🤖 Reinforcement Learning

### Training

```bash
python train.py --timesteps 500000 --envs 4
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--timesteps` | 100000 | Total RL training steps |
| `--envs` | 4 | Parallel environments |

**Algorithm:** PPO (Proximal Policy Optimization) via Stable-Baselines3

**Reward function:**
- `+forward_velocity × 10.0` — reward for moving in −Y direction
- `+0.5` — survival reward per step
- `−0.001 × Σ(action²)` — torque efficiency penalty
- `−1.0 × (|roll| + |pitch|)` — orientation stability penalty
- `−10.0` — termination penalty (fall / tilt > 0.8 rad / height < 0.05 m)

### Monitor Training

```bash
tensorboard --logdir ./tensorboard_logs
```

### Observation Space (37 dimensions)

| Component | Dims | Description |
|-----------|------|-------------|
| Euler orientation | 3 | Roll, Pitch, Yaw |
| Angular velocity | 3 | Body angular rates |
| Linear velocity | 3 | Body linear velocity |
| Joint positions | 12 | All 12 revolute joints |
| Joint velocities | 12 | All 12 joint velocities |
| Foot contacts | 4 | Binary contact per foot |

### Action Space (12 dimensions)

Normalized joint position targets in `[−1.0, 1.0]`, scaled to each joint's URDF limits.

---

## ⚙️ Kinematics

### Leg DOF

Each leg has **3 revolute joints**:

```
main_body
  └─ coxa_joint  (Z-axis rotation, yaw/sweep)
       └─ femur_joint  (Y-axis rotation, pitch up/down)
            └─ tibia_joint  (Y-axis rotation, pitch up/down)
                 └─ foot (fixed sphere, contact point)
```

### Diagonal Trot Gait

Locomotion uses a **diagonal trot** pattern:
- **Pair 1** (swing phase A): `fl` + `rr`
- **Pair 2** (swing phase B): `rl` + `fr`

The two pairs oscillate 180° out of phase. Coxa sweeps generate forward/backward/lateral motion; femur/tibia reduce angle during swing to lift the foot.

---

## 🏗️ Physics Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Simulation timestep | 1/240 s | PyBullet standard |
| Gravity | −9.81 m/s² | Earth gravity |
| Floor friction (lateral) | 0.8 | Rubber/concrete |
| Floor friction (spinning) | 0.05 | |
| Floor friction (rolling) | 0.02 | |
| Foot friction (lateral) | 1.5 | Rubber tip |
| Foot friction (spinning) | 0.3 | |
| Contact stiffness (foot) | 5000 | |
| Contact damping (foot) | 100 | |
| Joint damping | 0.05 | All revolute joints |
| Joint friction | 0.01 | All revolute joints |
| Coxa effort limit | 5.0 N | |
| Femur/Tibia effort limit | 6.0 N | |

---

## 📄 License

This project is open-source. See [LICENSE](LICENSE) for details.

---

## 🙋 Author

**Fitranur Mayadi**  
📧 GitHub: [@fitranurmayadi](https://github.com/fitranurmayadi)
