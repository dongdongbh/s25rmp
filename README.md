# s25rmp: Robotic Motion Planning for Poppy Ergo Jr

This repository contains code and configuration for developing a Task & Motion Planner (TAMP) for the Poppy Ergo Jr robotic arm using PyBullet, PDDLStream, and optional OMPL motion planning.

---
## 📁 Repository Structure

```
├── evaluation.py             # Scripts for benchmarking and metrics
├── example.py                # Minimal example usage
├── LICENSE
├── pddl/                     # PDDL domain & problem definitions
│   ├── domain.pddl           # Symbolic actions: pick/place/move
│   └── problem.pddl          # Initial & goal state for stacking blocks
├── planners/                 # Planner interface code
│   ├── ompl_motion.py        # RRT-Connect wrapper using OMPL
│   └── pddlstream_interface.py # Stream samplers for IK, motion, collision
├── poppy_ergo_jr.pybullet.urdf # Robot URDF description
├── README.md                 # Project overview & setup instructions
├── requirements.txt          # Python dependencies
├── run_planner.py            # Entry point: PDDLStream planning & execution
├── simulation.py             # Standalone PyBullet simulation & visualization
├── submission.py             # (Template) final submission script
```

---
## 🚀 Quickstart: Environment Setup

1. **Clone & Meshes**
   ```bash
   git clone https://github.com/your-org/s25rmp.git
   cd s25rmp
   # Download `meshes.zip` from SU OneDrive and extract:
   unzip meshes.zip -d meshes
   ```

2. **Create Conda environment**
   ```bash
   conda create -n s25rmp python=3.8 -y
   conda activate s25rmp
   ```

3. **Install Python packages**
   ```bash
   pip install -r requirements.txt
   ```

4. **Build & Install PDDLStream** (if using PDDLStream)
   ```bash
   git clone --recursive --branch main git@github.com:caelan/pddlstream.git
   cd pddlstream
   ./downward/build.py
   
   # Create a setup.py to make PDDLStream importable
   cat <<EOF > setup.py
   from setuptools import setup, find_packages

   setup(
       name='pddlstream',
       version='0.1',
       packages=find_packages(),
   )
   EOF
   
   # Install in editable mode
   pip install -e .

   cd ../s25rmp
   export PYTHONPATH=$PYTHONPATH:$(pwd)/../pddlstream
   ```

5. **(Optional) OMPL**
   ```bash
   conda install -c conda-forge ompl
   ```

---
## 🏃‍♂️ Running the Simulation

```bash
python simulation.py
```

---
## 🤖 Running the Planner

```bash
python run_planner.py
```
---
## 🗂️ Project Outline & Task Division

We have three main modules, each owned by one team member:

1️⃣ **Symbolic Task Planner (PDDLStream)**
- **Owner:** Teammate A
- **Responsibilities:**
  - Write `pddl/domain.pddl` (types, predicates, pick/place/move actions).
  - Write `pddl/problem.pddl` (initial state, goal arrangement).
  - Configure `planners/pddlstream_interface.py` to map symbolic preconditions to streams (e.g., `ik-solved`, `motion-solved`, `traj-free`).
  - Invoke PDDLStream’s `solve` in `run_planner.py` and parse the resulting sequence of actions + stream bindings.

2️⃣ **IK & Collision Module**
- **Owner:** Teammate B
- **Responsibilities:**
  - Use HW2 FK/IK code to implement `ik_stream(block, grasp) -> config` in Python.
  - Implement collision checking (`cfree_config(q)`) and carried-block collision along a trajectory (`traj_free(path)`) using python-fcl or a DIRECT-mode PyBullet client.
  - Expose these as PDDLStream streams so the planner can lazily verify feasibility.

3️⃣ **Motion Planning Module**
- **Owner:** Teammate C
- **Responsibilities:**
  - Wrap existing `goto_position` (linear interpolation) as `motion_stream(q1, q2)` for free-space motions.
  - Integrate OMPL RRT-Connect fallback by implementing `plan_rrt_connect(q1, q2)` in `planners/ompl_motion.py`.
  - In `motion_stream`, first attempt interpolation; if that fails collision checks, call RRT-Connect to generate a collision-free path.

This three-way split ensures clear ownership:
- **A** crafts the high-level plan,
- **B** guarantees each configuration and trajectory is collision-free,
- **C** provides fast and reliable motion paths (interpolation + sampling-based fallback).

