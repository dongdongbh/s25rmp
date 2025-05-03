# s25rmp: Poppy Ergo Jr Task‐and‐Motion Planning

**A modular TAMP pipeline** for the Poppy Ergo Jr arm, combining:

* **Symbolic Task Planning** via PDDLStream
* **Continuous Feasibility Checks** (IK, collision, motion) in PyBullet forks
* **Dynamic Location Generation** for stacking levels
* **Plan–Validate–Execute Loop** to ensure real‐world feasibility

---

## 📂 Project Structure

```
├── pddl/                         # PDDL domain & problem definitions
│   ├── domain.pddl               # types, predicates, actions (pick/move/place)
│   └── problem.pddl              # problem template (initial & goal)
├── planners/                     # Planning‐interface modules
│   ├── pddlstream_interface.py   # forked‐env streams: ik, cfree_config, motion, traj_free, gen_loc
│   └── ompl_motion.py            # RRT‑Connect wrapper
├── simulation.py                 # SimulationEnvironment (PyBullet API)
├── run_planner.py                # plan_task() wrapper around PDDLStream.solve
├── submission.py                 # Controller.run(): plan–validate–execute loop
├── evaluation.py                 # end‐to‐end evaluation: accuracy & error metrics
├── example.py                    # minimal demo controller usage
├── tests/                        # unit tests
│   ├── test_streams.py           # IK/collision/trajectory tests
│   └── test_motion.py            # motion planning tests
├── requirements.txt              # pip dependencies
└── README.md                     # this file
```

---

## 🚀 Quickstart

1. **Clone & resources**

   ```bash
   git clone https://github.com/your-org/s25rmp.git
   cd s25rmp
   # ensure URDF and meshes are under ./meshes/
   ```

2. **Python environment**

   ```bash
   conda create -n s25rmp python=3.8 -y
   conda activate s25rmp
   pip install -r requirements.txt
   ```

3. **PDDLStream**

   ```bash
   cd ..
   git clone --recursive --branch main git@github.com:caelan/pddlstream.git
   cd pddlstream && ./downward/build.py
   pip install -e .
   cd ../s25rmp
   export PYTHONPATH=$PYTHONPATH:$(pwd)/../pddlstream
   ```

4. **OMPL**

   ```bash
   conda install -c conda-forge \
    cmake \
    boost \
    eigen \
    swig \
    python \
    numpy
 
    pip install pyplusplus
     
    git clone https://github.com/ompl/ompl.git
    cd ompl
    mkdir build && cd build
    cmake .. \
      -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
      -DPYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python \
      -DOMPL_BUILD_PYBINDINGS=ON
     
    make -j$(nproc) update_bindings
     
    make install
   ```

---

## 🏃‍♂️ Running

* **Simulation only**:

  ```bash
  python simulation.py
  ```
* **Planner only** (one‐shot planning with forked‐env streams):

  ```bash
  python run_planner.py
  ```
* **Minimal demo**:

  ```bash
  python example.py
  ```
* **Full evaluation**:

  ```bash
  python evaluation.py
  ```
* **Unit tests**:

  ```bash
  pytest tests/
  ```

---

## 🧩 New Design Highlights

### 1️⃣ Fork‑per‑stream Architecture

Each PDDLStream stream (*ik*, *cfree\_config*, *motion*, *traj\_free*) now:

* **Resets** to a clean robot+floor snapshot, re‑adds all blocks
* **Runs** IK/collision/path tests in isolation

This guarantees correctness without persistent simulator side‑effects and keeps calls fast via DIRECT mode.

### 2️⃣ Held‑Block Filtering

`traj_free` filters out robot–held‐block contacts so that carrying an object doesn’t trigger spurious collisions:

```python
contacts = pb.getContactPoints(bodyA=env.robot_id)
filtered = [c for c in contacts if c[2] != held_id]
if not filtered:
    yield ()
```

### 3️⃣ Dynamic Location Generation

`gen_loc_stream` lazily creates `baseA_loc0`, `baseA_loc1`, … as needed, avoiding pre‑allocating a fixed stack size.

### 4️⃣ Plan–Validate–Execute Loop

`submission.py`’s `Controller.run`:

1. **Snapshot** real world → build in‑memory `PDDLProblem`
2. **One‑shot solve** via PDDLStream
3. **Step through** returned plan:

   * **Validate** each action with the same stream fns (forked env)
   * **Execute** in real PyBullet if valid; else **replan** from current state

This combines fast discrete planning with robust continuous validation.

---

## 📚 Module Breakdown

| Module                 | File(s)                              | API                                            |
| ---------------------- | ------------------------------------ | ---------------------------------------------- |
| Symbolic Planner       | `pddl/domain.pddl`, `run_planner.py` | `plan_task(domain, problem, STREAMS, ACTIONS)` |
| IK & Collision Streams | `planners/pddlstream_interface.py`   | `ik_stream`, `cfree_config`, `traj_free`       |
| Motion Planning        | `planners/ompl_motion.py`            | `plan_rrt_connect`, `motion_stream`            |
| Controller Loop        | `submission.py`                      | `Controller.run(real_env, goal_poses)`         |

---

## 🔧 Implementation Notes

* **WorldState**: `Dict[block, (pos, quat)]` passed as `?w` to each stream
* **Forked env**: built by `_make_env_from_world(world)` in `pddlstream_interface.py`
* **Held‐block**: streams receive `?b` so they can filter its body ID
* **Static vs. dynamic**:

  * Static facts (`Base`, `Location`, `Empty`, known `(Kin)` seeds) go in `problem.pddl`
  * Dynamic block positions come from `extract_physical_world(env)` in `submission.py`

---

Happy stacking! 🚀

