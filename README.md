# Robot Simulation MATLAB GUI

[![CI](https://github.com/mertcan-kaya/robot-simulation-matlab-gui/actions/workflows/ci.yml/badge.svg)](https://github.com/mertcan-kaya/robot-simulation-matlab-gui/actions/workflows/ci.yml)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Toolbox](https://img.shields.io/badge/Toolbox-v1.0.0--beta.1-orange.svg)](releases/Robot_Simulation_GUI.mltbx)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A modern, object-oriented **MATLAB App Designer** application for 3D modeling, kinematics, dynamics, trajectory planning, and control simulation of serial robotic manipulators.

![Robot Simulation GUI](docs/screenshot.png)

---

##  Key Features

* **3D Visual Simulation**:
  * Real-time 3D rendering with CAD STL meshes.
  * Hierarchical forward transformations using optimized `hgtransform` scene graphs.
  * Interactive coordinate frame display (Base, Joints, Flange, End-Effector).
  * Ghost robot rendering for initial/target visualization.
  * Real-time end-effector trajectory and trace lines.

* **Multi-Robot Support & Custom Modeling**:
  * Pre-configured industrial and collaborative robots (Franka Emika Panda, Universal Robots UR3, Unitree Z1, Stäubli RX160 / RX160L).
  * **Custom Robot Designer**: Define arbitrary $N$-DoF serial manipulators with user-specified Denavit-Hartenberg (DH) parameters, mass properties, inertias, and revolute/prismatic joints.

* **Kinematics Engine**:
  * Forward Kinematics (FK) with configurable Euler Angle conventions (ZYZ, ZYX, XYZ).
  * Inverse Kinematics (IK) supporting:
    * **Numerical Jacobian IK** (Damped Least Squares & Pseudoinverse).
    * **Analytical Closed-Form IK** for 6-DoF spherical wrist manipulators (UR3, RX160).
    * **Hybrid Geometric IK**.
  * Joint and task space limit monitoring with safety clamps.

* **Trajectory Planning**:
  * Point-to-point interpolation profiles:
    * **Linear** ($C^0$ continuous)
    * **Cubic Polynomial** ($C^1$ smooth)
    * **Quintic Polynomial** ($C^2$ smooth, continuous jerk)
    * **Trapezoidal Velocity** (Bang-bang acceleration)
  * Dynamic calculation of minimum execution time based on joint velocity and acceleration limits.

* **Dynamics & Control Simulation**:
  * Forward dynamics simulation via Recursive Newton-Euler Algorithm (RNEA).
  * **Inverse Dynamics Control (IDC / Computed Torque)**.
  * **Joint PID Control** with anti-windup.
  * **Gravity Compensation** algorithms.
  * Nonlinear joint friction modeling (Viscous, Coulomb, Stribeck, and FER empirical models).
  * Passive joint spring stiffness modeling.

* **Clean Object-Oriented Architecture**:
  * Strict Model-View-Controller (MVC) separation.
  * Standalone headless `+robotics` backend package callable from scripts, unit tests, or other GUIs.

---

##  Supported Robots

| Robot Model | Manufacturer | DoF | Joint Types | Features |
| :--- | :--- | :---: | :---: | :--- |
| **Franka Emika Panda** | Franka Robotics | 7 | Revolute | 7-DoF redundant arm with optional 2-finger parallel gripper |
| **Universal Robots UR3** | Universal Robots | 6 | Revolute | 6-DoF collaborative manipulator with analytical IK |
| **Unitree Z1** | Unitree | 6 | Revolute | Lightweight dexterous robotic arm |
| **Stäubli RX160** | Stäubli | 6 | Revolute | High-precision industrial manipulator |
| **Stäubli RX160L** | Stäubli | 6 | Revolute | Extended-reach industrial manipulator |
| **Custom Robot** | *User Defined* | $N$ | Rev / Prism | Fully configurable DH parameters, masses, and inertia tensors |

---

##  Getting Started

### Prerequisites
* MATLAB R2020b or later.
* *(Optional)* MATLAB Automated Testing Framework for running unit tests.

### Installation

#### Option 1: MATLAB Toolbox Installation (Recommended)
1. Double-click [`releases/Robot_Simulation_GUI.mltbx`](releases/Robot_Simulation_GUI.mltbx) in MATLAB.
2. Click **Install**.
3. Launch the app directly by running `MainApp` in the MATLAB Command Window or from the **Apps** tab.

#### Option 2: Run from Source
1. Clone or download this repository:
   ```bash
   git clone https://github.com/mertcan-kaya/robot-simulation-matlab-gui.git
   ```
2. Open MATLAB and navigate to the project directory.
3. Open [`MainApp.mlapp`](MainApp.mlapp) in App Designer, or run:
   ```matlab
   MainApp
   ```

---

##  Usage Guide

### 1. Joint Space & Task Space Manipulation
* **Joint Space Tab**: Adjust individual joint angles using sliders or numeric spinners.
* **Task Space Tab**: Set initial and target positions $(X, Y, Z)$ and orientation Euler angles $(\phi, \theta, \psi)$.
* **Forward/Inverse Kinematics**: Sliders automatically synchronize in real time.

### 2. Trajectory Configuration & Execution
1. Select your desired interpolation profile from the dropdown (**Linear**, **Cubic**, **Quintic**, or **Trapezoidal**).
2. Set velocity and acceleration scaling percentages. The planner automatically calculates the minimum safe execution time $t_f$.
3. Choose **Kinematic** or **Dynamic** simulation mode.
4. Click **Run** to visualize the motion.

### 3. Custom Robot Modeling
1. Select **Custom Robot** from the robot dropdown.
2. Click **Edit** to enter the custom parameter dialog.
3. Define the number of degrees of freedom $N$, link lengths, twist angles, joint offsets, link masses, and inertia matrices.
4. The 3D scene and kinematics engines adapt dynamically to your custom manipulator.

---

## ️ Architecture & Project Structure

The project follows a modular, package-based Model-View-Controller (MVC) design pattern:

```
robot-simulation-matlab-gui/
├── +robotics/
│   ├── +engines/         # Physics, Kinematics, Dynamics, and Simulation Controllers
│   │   ├── DynamicsEngine.m       # Forward dynamics & RNEA inverse dynamics
│   │   ├── KinematicsEngine.m     # FK, Analytical IK, and Numerical IK
│   │   ├── TrajectoryEngine.m     # P2P trajectory generation
│   │   ├── ControlEngine.m        # PID, Computed Torque, and Gravity Comp
│   │   ├── SimulationEngine.m     # Time-stepping simulation loop
│   │   └── SimulationController.m # Core controller mediating Model and View
│   ├── +friction/        # Joint friction and spring models (Viscous, Coulomb, FER)
│   ├── +graphics/        # 3D visualization, MeshLoader, and PlotAxesManager
│   ├── +math/            # SE(3), SO(3), Skew-symmetric, and Euler conversion routines
│   ├── +models/          # OOP Robot classes (Franka, UR3, Z1, RX160, CustomRobot, SimulationModel)
│   ├── +trajectory/      # Trajectory strategy planners (Linear, Cubic, Quintic, Trapezoidal)
│   └── +viewmodels/      # UI-agnostic presentation and string formatting layer
├── docs/                 # Documentation assets and architecture guides
│   ├── ARCHITECTURE.md   # Detailed developer architecture specification
│   └── screenshot.png    # GUI preview image
├── meshes/               # CAD STL 3D models for link rendering
├── releases/             # Pre-built MATLAB Toolbox (.mltbx) packages
├── tests/                # Automated unit test suite
│   ├── TestInverseKinematics.m
│   ├── TestRobotModels.m
│   └── TestTrjGeneration.m
├── MainApp.mlapp         # MATLAB App Designer application
├── MainApp_code.txt      # Reference source code for App Designer Code View
├── package_toolbox.m     # Toolbox packaging automation script
├── runAllTests.m         # Unit test runner
└── README.md             # Project documentation
```

For more details on the software architecture and extension points, see [ARCHITECTURE.md](docs/ARCHITECTURE.md).

---

##  Running Unit Tests

Run the automated test suite directly from the MATLAB command window:

```matlab
runAllTests
```

The test suite validates:
* Object-Oriented Robot Models & parameter dimensions across all DoFs.
* Analytical and Numerical Inverse Kinematics accuracy.
* Trajectory generation continuity and boundary condition satisfaction.

---

##  Building the Toolbox

To build the standalone `.mltbx` installer:

```matlab
package_toolbox
```

The packaged toolbox will be generated inside the [`releases/`](releases/) directory.

---

##  Screenshots

### Kinematics & 3D Visualization
![Kinematics](https://github.com/user-attachments/assets/4888eb6b-43e1-4714-b677-ca2f5cb90f73)

### Joint Limits Configuration
![Joint Limits](https://github.com/user-attachments/assets/bb2669dd-995a-442d-814d-b5f3cd92bff0)

### Inertial & Dynamic Parameters
![Inertial Parameters](https://github.com/user-attachments/assets/aa8cbdc7-0cb6-4bb9-a6f2-ba1c320ec579)

### Control Algorithms (IDC / PID)
![Control Parameters](https://github.com/user-attachments/assets/2ab55a2d-55fd-4231-8d1b-16ea74390250)

### Custom Robot Modeling
![Custom Robot Modeling](https://github.com/user-attachments/assets/a5001fe7-1e59-40e9-bf32-823c705d6a15)

---

##  License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

##  Author

**Mertcan Kaya**
* GitHub: [@mertcan-kaya](https://github.com/mertcan-kaya)
