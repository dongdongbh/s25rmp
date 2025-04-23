# s25rmp: Poppy Ergo Jr Motion Planning

**This repository implements a full Task‐and‐Motion Planning (TAMP) pipeline** for the Poppy Ergo Jr platform.  We combine:

- **Symbolic Task Planning** with PDDLStream
- **Inverse Kinematics & Collision Checking** (HW2 code + FCL or PyBullet)
- **Motion Planning** (linear interpolation + OMPL RRT‑Connect fallback)

---
## 📂 Project Structure

```
├── pddl/                         # PDDL domain & problem definitions
│   ├── domain.pddl               # :action pick/place/move
│   └── problem.pddl              # initial & goal block states
├── planners/                     # Planning‐interface modules
│   ├── pddlstream_interface.py   # ik_stream, cfree_config, traj_free, STREAMS & ACTIONS
│   └── ompl_motion.py            # plan_rrt_connect() wrapper for OMPL RRT‑Connect
├── simulation.py                 # SimulationEnvironment API (PyBullet)
├── run_planner.py                # plan_task() wrapper for PDDLStream
├── submission.py                 # Controller.run() — integrate planner → env
├── evaluation.py                 # End‐to‐end evaluation of Controller (accuracy metrics)
├── example.py                    # Minimal example of Controller usage
├── tests/                        # Unit tests for each module
│   ├── test_streams.py
│   └── test_motion.py
├── requirements.txt              # pip dependencies
├── README.md                     # This file
└── LICENSE
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

4. **Build & Install PDDLStream** 
    
   ```bash
   cd ..
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

---
## 📂 Project Structure & Responsibilities

We split the system into three clear modules—each with defined files, APIs, and tests—so teammates can work in parallel:

| Module                          | Owner       | Key Files                          | Public API                                | Unit Tests                              |
|---------------------------------|-------------|------------------------------------|-------------------------------------------|------------------------------------------|
| **Symbolic Task Planner**       | Teammate A  | `pddl/domain.pddl`, `pddl/problem.pddl`, `run_planner.py`  | `plan_task(domain, problem, STREAMS, ACTIONS) -> List[(action, params)]` | `tests/test_task_planner.py`            |
| **IK & Collision**              | Teammate B  | `planners/pddlstream_interface.py`, `simulation.py` | `ik_stream(block, pose) -> Iterator[(config,)]`  <br> `cfree_config(config) -> bool`  <br> `traj_free(path, block) -> Iterator[()]` | `tests/test_streams.py`                  |
| **Motion Planning**             | Teammate C  | `planners/ompl_motion.py`<br>`planners/pddlstream_interface.py` | `motion_stream(q1, q2) -> Iterator[(path,)]`  <br> `plan_rrt_connect(q1, q2) -> Optional[path]` | `tests/test_motion.py`                  |

---
## 🔌 Module Interfaces & Implementation Details

### 1️⃣ Symbolic Task Planner
**Files:** `pddl/domain.pddl`, `pddl/problem.pddl`, `run_planner.py`

```python
# run_planner.py
from pddlstream.algorithms.meta import solve
from pddlstream.language.stream import StreamInfo
from pddlstream.language.function import FunctionInfo
from typing import Dict, List, Tuple

def plan_task(
    domain_pddl: str,
    problem_pddl: str,
    streams: Dict[str, StreamInfo],
    actions: List[FunctionInfo],
    planner: str = 'ff-astar',
    algorithm: str = 'adaptive'
) -> List[Tuple[str, Dict]]:
    """Return [ (action_name, {param: value}), ... ]."""
    return solve(
        domain_pddl=domain_pddl,
        problem_pddl=problem_pddl,
        streams=streams,
        actions=actions,
        planner=planner,
        algorithm=algorithm,
    )
```

**Testing:** Write `tests/test_task_planner.py` that asserts `plan_task(...)` returns expected `pick`/`place` sequence on a trivial 1-block problem.

---
### 2️⃣ IK & Collision Module
**Files:** `planners/pddlstream_interface.py`, (optionally `collision.py`)

```python
# planners/pddlstream_interface.py
from typing import Iterator, Tuple, List
from simulation import SimulationEnvironment
import pybullet as pb

Config = Tuple[float, ...]
Path = List[Config]

def ik_stream(block_id: str,
              grasp_pose: Tuple[float, ...]
) -> Iterator[Tuple[Config,]]:
    """Yield joint configurations for grasping `block_id` at `grasp_pose`."""
    # use HW2 IK or PyBullet.call, then yield configs
    ...

def cfree_config(config: Config) -> bool:
    """Return True if robot at `config` has no collisions."""
    ...

def traj_free(path: Path,
              block_id: str
) -> Iterator[()]:
    """Yield () if carrying `block_id` along `path` is collision-free."""
    ...

# Register streams:
STREAMS = {
    'ik': StreamInfo(..., fn=from_fn(ik_stream)),
    'cfree_config': StreamInfo(..., fn=from_fn(cfree_config)),
    'traj_free': StreamInfo(..., fn=from_fn(traj_free)),
}
# Action schemas:
ACTIONS = [FunctionInfo('pick', ...), FunctionInfo('move', ...), FunctionInfo('place', ...)]
```

**Testing:** In `tests/test_streams.py`:
- `ik_stream`: yields >=1 config that passes `cfree_config`.
- `cfree_config`: True on free config, False on known collision.
- `traj_free`: yields `()` only on collision-free path.

---
### 3️⃣ Motion Planning Module
**Files:** `planners/ompl_motion.py`, enhancements in `planners/pddlstream_interface.py`

```python
# planners/ompl_motion.py
from typing import List, Optional
Config = Tuple[float, ...]
Path = List[Config]

def plan_rrt_connect(
    q1: List[float], q2: List[float], timeout: float = 1.0
) -> Optional[Path]:
    """Return a collision-free path or None."""
    ...

def motion_stream(
    q1: Config, q2: Config
) -> Iterator[Tuple[Path,]]:
    """Interpolate, else fallback to RRT-Connect."""
    # 1) linear interpolation + cfree_config check
    # 2) if fails, plan_rrt_connect
    ...
```

**Testing:** In `tests/test_motion.py`:
- Check `motion_stream` yields correct path start/end.
- For obstructed configs, assert fallback to `plan_rrt_connect`.

---
## ✅ Full Integration & End‐to‐End Test

- **Controller** in `submission.py` stitches together `plan_task`, then calls `env.goto_position`, `env._step`, and `_add_block` as needed.
- Run **end‐to‐end**: `python evaluation.py` prints stacking accuracy and errors.

---
*By following this unified framework, each teammate can develop, test, and integrate their component seamlessly.*

## 🏃‍♂️ Usage

```bash
python simulation.py        # visualize arm
python example.py           # demo controller
python run_planner.py       # run PDDLStream+sim
pytest tests/               # unit tests
python evaluation.py        # full evaluation
```

- **Unit tests** in `tests/`:
  ```bash
  pytest tests/
  ```
- **End-to-end** simulation:
  ```bash
  python evaluation.py
  ```
  Reports % of blocks correctly stacked and error metrics.

---
## 🏃‍♂️ Usage Examples

- **Standalone simulation**: `python simulation.py`
- **Minimal demo**: `python example.py`
- **Planner + sim**: `python run_planner.py`
- **Full eval**: `python evaluation.py`

---
*Fill in STREAMS & ACTIONS, implement the stub functions above, then test!*


