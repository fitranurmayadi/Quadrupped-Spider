# Quadruped Spider Robot 🕷️

A **4-legged quadruped spider robot** simulation and locomotion control workspace built with **PyBullet**. The 3D model is designed in **FreeCAD** and exported as STL meshes for physics simulation.

This project implements a high-performance **Analytical Inverse Kinematics (IK)** solver and a **Diagonal Trot Gait Generator** with dynamic trajectory shaping, allowing stable conventional control without relying on Reinforcement Learning.

---

## 📋 Table of Contents

- [Features](#-features)
- [Project Structure](#-project-structure)
- [Coordinate System](#-coordinate-system)
- [Kinematics & Control](#-kinematics--control)
- [Installation](#-installation)
- [Usage](#-usage)
- [Locomotion Versions (V1 vs V2)](#-locomotion-versions-v1-vs-v2)
- [Testing](#-testing)
- [Physics Parameters](#-physics-parameters)

---

## ✨ Features

- **Realistic 3D model** — Body, coxa, femur, and tibia meshes exported from FreeCAD
- **PyBullet physics** — Accurate rigid-body dynamics with joint damping, friction, and contact forces
- **Analytical Inverse Kinematics (IK)** — Custom 3-DOF per-leg closed-form solver with perfect reconstruction accuracy ($0.00$ m error)
- **Smooth Trajectory Planning** — Parabolic vertical swing curves and horizontal cosine acceleration profiles to prevent joint impact shocks
- **Stance Overlap (Double Support)** — Configurable duty factor (e.g. 55%) ensuring a stable 4-foot stance phase during step transitions
- **Dynamic Gait Scaling** — Automatically scales down step size and increases stance overlap as body height increases to stabilize the Center of Mass (CoM)
- **Interactive Keyboard Control** — Real-time keyboard control (WASD/Arrows for translation, Q/E for turning, Z/X for height) with a 3rd-person follow camera

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
├── spider_ik.py              # Analytical Inverse Kinematics solver
├── gait_controller.py        # Gait V1: Trot gait generator (standard splay)
├── gait_controller_v2.py     # Gait V2: Trot gait with vertical tibia & dynamic splay
│
├── testing/
│   ├── test_1_standing.py    # Test 1: Standing stability test
│   └── test_2_attitude.py    # Test 2: Attitude control & motion test
│
├── walk_demo.py              # Demo script to walk exactly 1.0 meter forward
├── keyboard_control.py       # Interactive keyboard control (Gait V1)
├── keyboard_control_v2.py    # Interactive keyboard control (Gait V2, Vertical Tibia)
│
├── display_robot.py          # Visualize robot in PyBullet GUI
├── interactive_calibration.py# Interactive joint calibration tool
├── basic_axis_check.py       # Coordinate axis alignment verification
├── requirements.txt          # Python dependencies
└── README.md
```

---

## 🧭 Coordinate System

This project uses **FreeCAD's default coordinate convention**:

| Direction | World Axis | Description |
|-----------|-----------|-------------|
| **Front** (Depan) | **−Y** | Robot faces negative Y |
| **Rear** (Belakang) | **+Y** | Back of robot |
| **Left** (Kiri) | **+X** | Robot's left side |
| **Right** (Kanan) | **−X** | Robot's right side |
| **Up** | **+Z** | Vertical up |

---

## ⚙️ Kinematics & Control

### Leg Degrees of Freedom (12 DOF Total)

Each of the 4 legs has **3 revolute joints**:
```
main_body
  └─ coxa_joint  (Z-axis rotation, yaw/sweep)
       └─ femur_joint  (Y-axis rotation, pitch up/down)
            └─ tibia_joint  (Y-axis rotation, pitch up/down)
                 └─ foot (fixed sphere, contact point)
```

### Analytical Inverse Kinematics

The solver in `spider_ik.py` computes joint angles ($\theta_1, \theta_2, \theta_3$) for any foot target position $(x, y, z)$ relative to the leg base:
1. **Coxa angle ($\theta_1$)**: $\text{atan2}(y, x)$
2. **Femur ($\theta_2$) & Tibia ($\theta_3$)**: Solved using 2-Link Planar IK geometry in the leg plane:
   $$\theta_2 = -\phi_2, \quad \theta_3 = -\phi_3$$
   Where $\phi_3 = \text{atan2}(-\sqrt{1-D^2}, D)$ handles the elbow-down orientation.

---

## 🔧 Installation

### Prerequisites

- Python 3.9+

### Steps

```bash
git clone https://github.com/fitranurmayadi/Quadrupped-Spider.git
cd Quadrupped-Spider
pip install -r requirements.txt
```

---

## 🚀 Usage

### 🎮 Interactive Keyboard Control (Recommended)

Run the **Vertical Tibia V2** interactive controller:
```bash
python keyboard_control_v2.py
```
*Make sure to click/focus the PyBullet GUI window to capture keyboard input.*

**Key Mappings:**
- **`W` / `S` / `A` / `D`** (or **Arrow Keys**): Translate Maju, Mundur, Kiri, Kanan
- **`Q` / `E`**: Rotate Turn Left / Turn Right
- **`Z` / `X`**: Raise / Lower Body Height (1.5 cm to 7.5 cm)
- **`R`**: Reset Robot Position & Height
- **`ESC`**: Exit Simulation

---

## 🔄 Locomotion Versions (V1 vs V2)

### 🐾 Gait V1 (`gait_controller.py` & `keyboard_control.py`)
- **Gait**: standard diagonal trot.
- **Splay**: Fixed footprint width.
- **Camera**: Top-down view.

### 🐾 Gait V2 (`gait_controller_v2.py` & `keyboard_control_v2.py`)
- **Vertical Tibia**: Femur and Tibia angles are automatically constrained to keep the tibia link at exactly **$90^\circ$ vertical** to the ground when standing.
- **Dynamic Footprint Splay**: The horizontal foot placement distance ($r$) is calculated dynamically from the height:
  $$r = \sqrt{L_2^2 - (z_{\text{proj}} + L_3)^2}$$
  - Raising the body (**`Z`**) pulls the legs inward.
  - Lowering the body (**`X`**) spreads the legs outward.
- **Stance Overlap (Double Support)**: `duty_factor` of 0.55 ensures all four feet remain on the ground for 10% of the cycle time during step transitions.
- **Cosine Swing Profile**: Reduces foot horizontal speed to zero before touchdown to prevent hard impacts.
- **Dynamic Gait Scaling**: Shuts down step length and height (down to 50%) at high heights to stabilize the Center of Mass (CoM).
- **Camera**: Low-angle third-person tracking view.

---

## 🧪 Testing

### Walk 1 Meter forward
Runs the robot forward exactly 1.0 meter and terminates.
```bash
python walk_demo.py --headless
```

### Standing Stability Check
```bash
python testing/test_1_standing.py --headless
```

---

## 🏗️ Physics Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Simulation timestep | 1/240 s | PyBullet standard |
| Gravity | −9.81 m/s² | Earth gravity |
| Floor friction (lateral) | 0.8 | Concrete/Rubber |
| Foot friction (lateral) | 1.5 | High-grip rubber tips |
| Joint damping | 0.05 | Damping for all joints |
| Joint friction | 0.01 | Friction for all joints |

---

## 🙋 Author

**Fitranur Mayadi**  
📧 GitHub: [@fitranurmayadi](https://github.com/fitranurmayadi)
