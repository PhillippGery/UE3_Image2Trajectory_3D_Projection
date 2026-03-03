
# Robotic Arm Trajectory Control

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

A MATLAB + ROS2 pipeline for controlling a **6-DOF robotic arm** to autonomously execute custom end-effector trajectories using screw-theory-based kinematics, trapezoidal velocity profiling, and image-to-path conversion.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Demo](#demo)
- [Quickstart](#quickstart)
- [System Architecture](#system-architecture)
- [Pipeline](#pipeline)
- [Key Algorithms](#key-algorithms)
- [MATLAB Modules](#matlab-modules)
- [ROS2 Integration](#ros2-integration)
- [Results](#results)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)

---

## Overview

This project implements end-to-end trajectory control for a 6-DOF robotic arm, with the primary goal of **drawing 3D light projections in mid-air using an LED end-effector**. The robot autonomously traces any image or signature as a luminous path through space — captured by long-exposure photography to reveal the full 3D projection. As a demonstration, the robot draws a custom signature/logo, but the pipeline is fully general and supports any 2D or 3D end-effector path.

The system:

- **Converts any image** into smooth, scaled robot-frame XY coordinates
- **Plans trapezoidal velocity profiles** for smooth, jerk-minimized motion
- **Computes forward and inverse kinematics** using modern robotics (Product of Exponentials / screw theory)
- **Verifies singularity avoidance** via Jacobian determinant monitoring throughout the trajectory
- **Generates joint angle trajectories** and exports to CSV for ROS2 hardware execution
- **Controls a binary end-effector output** (e.g. pen, LED, tool) synchronized to path segments

---

## Demo

The robot arm traces a custom signature in 3D space with an LED end-effector. The light path is captured via long-exposure photography, revealing the full trajectory as a glowing projection.

### Live Execution

![Demo GIF](demo.gif)

### Long-Exposure Light Projections

<p align="center">
  <img src="https://github.com/user-attachments/assets/0b7f7970-f63b-44d8-a445-03f283068bc2" width="48%" alt="3D Light Projection — circular logo"/>
  &nbsp;
  <img src="https://github.com/user-attachments/assets/c37b4a65-cec6-4e4c-952a-15d80a2b6adb" width="48%" alt="3D Light Projection — script signature"/>
</p>

> Long-exposure captures of the LED end-effector tracing the "PGery" signature trajectory in mid-air.

---

## Quickstart

### Prerequisites

- MATLAB R2023b+ with Image Processing Toolbox and Control System Toolbox
- ROS2 Humble (Ubuntu 22.04)
- Python 3.10+
- A ROS2-compatible 6-DOF robot arm (tested on UR5)

### 1 — Generate the Trajectory in MATLAB

```matlab
% Step 1: Convert your image to robot-frame paths
run('MATLAB/Image2Coordinates.m')
% → outputs: final_scaled_paths (cell array, meters)

% Step 2: Plan the trajectory and run IK
% For 2D (flat surface):
run('MATLAB/Bonous_constantV.m')

% For 3D (with Z offset / mid-air):
run('MATLAB/Image2Path_with_offset.m')
% → outputs: trajectory.csv  [t, θ1..θ6, output]
```

> To use your own image: replace `input_image.png` in the `MATLAB/` folder and adjust the `target_width` scale parameter inside `Image2Coordinates.m`.

### 2 — Copy the CSV to the ROS2 Workspace

```bash
cp MATLAB/trajectory.csv ws_robot/src/py_joint_pub/py_joint_pub/resource/
```

### 3 — Build and Run the ROS2 Node

```bash
cd ws_robot
colcon build --symlink-install
source install/setup.bash

ros2 run py_joint_pub joint_publisher_csv
```

The node will begin publishing joint states at **500 Hz** to `/joint_states`. The robot arm will execute the trajectory and loop continuously. The `output` column in the CSV drives the LED/tool on or off per segment.

### 4 — Verify (Optional)

```bash
# Monitor joint states in real-time
ros2 topic echo /joint_states

# Check publish rate
ros2 topic hz /joint_states
```

---

## System Architecture

<details>
<summary>Click to expand architecture diagram</summary>

```
┌──────────────────────────────────────────────────────────────────┐
│                        MATLAB Pipeline                           │
│                                                                  │
│  Input Image (PNG)                                               │
│       │                                                          │
│       ▼                                                          │
│  Image2Coordinates.m                                             │
│  ┌──────────────────────────────────────────────────┐            │
│  │ Binarize → Invert → Contour extraction           │            │
│  │ → Gaussian smooth → Simplify → Scale to meters   │            │
│  └──────────────────────────────────────────────────┘            │
│       │  final_scaled_paths (cell array, meters)                 │
│       ▼                                                          │
│  Trajectory Planner                                              │
│  ┌──────────────────────────────────────────────────┐            │
│  │ Build master path (transitions + active segments)│            │
│  │ Trapezoidal velocity profile v(t)                │            │
│  │ Arc-length re-timing → x(t), y(t), [z(t)]       │            │
│  │ Binary output status vector (ON/OFF per segment) │            │
│  └──────────────────────────────────────────────────┘            │
│       │                                                          │
│       ▼                                                          │
│  Forward Kinematics                                              │
│  → T0 at home configuration                                      │
│  → Tsd(t) = T0 × Td(t) at each timestep                         │
│       │                                                          │
│       ▼                                                          │
│  Inverse Kinematics (Newton-Raphson)                             │
│  → thetaAll(t): 6×N joint angle matrix                           │
│       │                                                          │
│       ▼                                                          │
│  Verification                                                    │
│  → FK re-check │ Jacobian det │ Joint continuity                 │
│       │                                                          │
│       ▼                                                          │
│  Export: trajectory.csv  [t, θ1..θ6, output]                    │
└──────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────────────┐
│                        ROS2 Node                                 │
│                                                                  │
│  joint_publisher_csv.py                                          │
│  → Reads trajectory.csv at 500 Hz                               │
│  → Publishes JointState to /joint_states                         │
│  → Robot arm executes trajectory in real-time                    │
└──────────────────────────────────────────────────────────────────┘
```

</details>

---

## Pipeline

### Step 1 — Image to Path Coordinates (`Image2Coordinates.m`)

Converts any input image into smooth, robot-frame coordinate paths:

1. Load image → grayscale → binarize
2. Auto-invert so foreground objects are detected correctly
3. Extract contours using `bwboundaries`
4. Smooth contours with a Gaussian filter
5. Simplify with `reducepoly` — reduces thousands of pixel points to a minimal smooth representation
6. Scale to a target width in meters and center at (0, 0) in robot frame
7. Save as `final_scaled_paths` cell array

This module is input-agnostic: any binary image (logos, signatures, technical drawings, geometric shapes) can be used as the trajectory source.

### Step 2 — Trajectory Planning

Builds a complete time-parameterized trajectory across all path segments:

- Assembles a **master path** interleaving transition segments (output OFF) and active segments (output ON)
- Computes total arc length across all 2D or 3D segments
- Generates a **trapezoidal velocity profile**:
  - Trapezoid profile if total distance ≥ `v_max × t_accel`
  - Triangle profile (reduced peak velocity) for short total distances
- Re-times the geometric path via arc-length interpolation: `d_desired(t) → x(t), y(t)`
- Generates a **binary output status vector** synchronized to the arc-length profile

**3D Extension:** Adds a configurable `drawing_offset [X, Y, Z]` for full 3D end-effector positioning, enabling execution on non-zero-plane surfaces (tilted boards, vertical walls, etc.).

### Step 3 — Forward Kinematics

Computes the home configuration transform `T0` using the **Product of Exponentials (PoE)** formula:

```
Space form: T(θ) = e^[S1]θ1 · e^[S2]θ2 · ... · e^[S6]θ6 · M
Body form:  T(θ) = M · e^[B1]θ1 · e^[B2]θ2 · ... · e^[B6]θ6
```

Kinematic parameters are read directly from the robot's URDF. Desired end-effector transform at each timestep: `Tsd(t) = T0 × Td(t)`

### Step 4 — Inverse Kinematics

Newton-Raphson iterative IK in the body frame:

```
θ_{i+1} = θ_i + J_b†(θ_i) · V_b
```

where `V_b` is the body-frame twist error derived from `MatrixLog6(T_sb⁻¹ · T_sd)` and `J_b†` is the Moore-Penrose pseudoinverse of the body Jacobian. Converges when `‖ω_b‖ < 1e-6` and `‖v_b‖ < 1e-6`.

The previous timestep solution is always used as the initial guess, ensuring smooth joint trajectories and avoiding configuration jumps between frames.

### Step 5 — Verification

Three checks before export:
- **FK re-validation**: Re-compute `actualTsd` from `thetaAll` via FK and compare to `Tsd`
- **Joint continuity**: Plot `diff(thetaAll)` — large jumps indicate IK discontinuities
- **Singularity monitoring**: Plot `det(J_b(θ))` over time — values near zero indicate kinematic singularities to avoid

### Step 6 — ROS2 Execution

Reads the exported CSV and publishes joint states at 500 Hz to `/joint_states`. Loops the trajectory continuously. Compatible with standard ROS2 robot drivers and MoveIt2.

---

## Key Algorithms

### Screw Theory Kinematics (Modern Robotics)
All kinematics use the **Product of Exponentials** formulation. Screw axes are defined in both space frame `{s}` and body frame `{b}`. The adjoint representation `[AdT]` maps twists between frames. This approach is numerically clean, singularity-representation-free, and directly tied to the Lie group structure of SE(3).

### Trapezoidal Velocity Profile
```
Phase 1 (Acceleration):  v(t) = (v_max / t_accel) × t
Phase 2 (Constant):      v(t) = v_max
Phase 3 (Deceleration):  v(t) = (v_max / t_accel) × (t_final - t)
```
Triangle profile activates automatically when total path distance is too short to reach `v_max`, ensuring the profile is always physically consistent.

### Arc-Length Re-timing
Geometric path `G(s)` (parameterized by arc length `s`) is re-timed by mapping `s = d_desired(t)` via `interp1`. This completely decouples path geometry from velocity profile — any velocity profile can be applied to any path without modifying the path definition.

### Binary Output Synchronization
Cumulative arc-length markers track segment boundaries. Output status is assigned per-timestep based on whether `d_desired(t)` falls within an active segment (even index → ON) or a transition segment (odd index → OFF). This approach naturally handles any number of path segments.

### Lissajous Reference Trajectory (`Des_Trajec.m`)
A Lissajous curve is included as a reference test trajectory for validating the kinematics pipeline independent of the image processing module:
```
x(t) = A · sin(a·t),   y(t) = B · sin(b·t)
```
Useful for benchmarking IK accuracy and velocity tracking before running complex image-derived paths.

---

## MATLAB Modules

| File | Purpose |
|---|---|
| `Image2Coordinates.m` | Image → smooth scaled robot-frame paths |
| `Bonous_constantV.m` | 2D trajectory planning + full IK pipeline |
| `Image2Path_with_offset.m` | 3D trajectory planning with configurable Z offset |
| `Des_Trajec.m` | Lissajous reference trajectory |
| `FKinSpace.m` | Forward kinematics (space frame PoE) |
| `FKinBody.m` | Forward kinematics (body frame PoE) |
| `IKinBody.m` | Newton-Raphson IK (body frame) |
| `JacobianBody.m` | Body Jacobian computation |
| `Adjoint.m` | 6×6 adjoint map [AdT] |
| `MatrixExp6.m` | Matrix exponential SE(3) |
| `MatrixLog6.m` | Matrix logarithm SE(3) |
| `MatrixExp3.m` | Matrix exponential SO(3) |
| `MatrixLog3.m` | Matrix logarithm SO(3) |
| `VecTose3.m` | 6-vector → 4×4 se(3) matrix |
| `VecToso3.m` | 3-vector → 3×3 so(3) matrix |
| `TransInv.m` | Efficient SE(3) inverse |
| `TransToRp.m` | Extract R, p from T |

---

## ROS2 Integration

```bash
# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Run the joint publisher
ros2 run py_joint_pub joint_publisher_csv
```

**CSV format:** `[t, θ1, θ2, θ3, θ4, θ5, θ6, output]`

The node publishes to `/joint_states` at 500 Hz and loops the trajectory continuously. The `output` column can be routed to any digital I/O topic for hardware tool control (pen, LED, gripper trigger, etc.).

---

## Results

### What Worked

- **Accurate trajectory reproduction**: The IK pipeline successfully converged for the full "PGery" signature trajectory with sub-millimeter FK re-validation error across all timesteps.
- **Smooth motion**: Trapezoidal velocity profiling produced visually smooth, jerk-minimized motion — no abrupt starts or stops observed during hardware execution.
- **LED synchronization**: The binary output vector correctly toggled the LED on/off at path segment boundaries, producing clean light traces with no bleed between strokes.
- **Long-exposure capture**: The LED end-effector approach proved highly effective for visualizing the 3D trajectory — the glowing projections matched the intended signature shape closely.
- **Pipeline generality**: The image-to-trajectory pipeline worked on multiple test inputs (logos, signatures, geometric shapes) without code changes.

### Known Limitations

- **Singularity sensitivity**: Certain input paths route the arm near kinematic singularities (det(J) ≈ 0), causing IK instability. Current mitigation is post-hoc monitoring via Jacobian determinant plots; active singularity avoidance is not yet implemented.
- **Planar constraint**: The primary pipeline operates in a fixed Z plane. True 3D surface following (curved surfaces, varying Z per-point) requires manual `drawing_offset` tuning per trajectory.
- **No collision avoidance**: The trajectory planner has no awareness of obstacles or self-collision; safe workspace bounds must be enforced manually via the input image scale and offset parameters.
- **Loop-only execution**: The ROS2 node loops the trajectory indefinitely with no single-shot or conditional stop mode currently exposed.
- **Camera-robot calibration**: Long-exposure results depend on precise camera placement and exposure settings; a formal extrinsic calibration between robot frame and camera was not implemented.

---

## Project Structure

```
Robotic_Arm_Trajectory/
├── MATLAB/
│   ├── Image2Coordinates.m        # Image → robot-frame paths
│   ├── Bonous_constantV.m         # 2D trajectory + IK pipeline
│   ├── Image2Path_with_offset.m   # 3D trajectory + IK pipeline
│   ├── Des_Trajec.m               # Lissajous reference trajectory
│   ├── input_image.png            # Example input (logo/signature/drawing)
│   ├── path_data.mat              # Saved scaled paths
│   ├── trajectory.csv             # Exported joint trajectory for ROS2
│   └── functions/
│       ├── Adjoint.m
│       ├── FKinBody.m / FKinSpace.m
│       ├── IKinBody.m
│       ├── JacobianBody.m
│       ├── MatrixExp3.m / MatrixExp6.m
│       ├── MatrixLog3.m / MatrixLog6.m
│       ├── TransInv.m / TransToRp.m
│       ├── VecTose3.m / VecToso3.m
│       └── NearZero.m / AxisAng3.m
└── ws_robot/
    └── src/
        └── py_joint_pub/
            └── py_joint_pub/
                ├── joint_publisher_csv.py   # ROS2 500Hz joint publisher
                └── resource/
                    └── trajectory.csv
```

---

## Dependencies

### MATLAB
- Image Processing Toolbox (`imbinarize`, `bwboundaries`, `reducepoly`)
- Control System Toolbox

### ROS2
- `rclpy` — ROS2 Python client library
- `sensor_msgs` — JointState message
- `numpy`

### Hardware
- 6-DOF robot arm (developed and tested on Universal Robots UR5)
- Compatible with any ROS2-supported manipulator via `ur_robot_driver` or equivalent

---

## Design Notes

The pipeline is intentionally modular: image processing, velocity planning, kinematics, and ROS2 execution are fully decoupled. This means any image can be used as input, any velocity profile can be swapped in, and the same MATLAB-generated CSV can drive any ROS2-compatible robot arm.

The 3D extension generalizes from planar drawing to arbitrary 3D surface trajectories using a single `drawing_offset` parameter, with no changes to the kinematics or velocity planning modules.

---

## Authors

**Phillipp Gery** — Purdue University, MS Interdisciplinary Engineering (Autonomy & Robotics)
Fulbright Scholar | GradBridge Program (Purdue & UC Berkeley)
