# s25rmp: Poppy Ergo Jr Task‐and‐Motion Planning

A modular TAMP pipeline for the Poppy Ergo Jr. arm, combining:

- **Symbolic Task Planning** via PDDLStream  
- **Continuous Feasibility Checks** (IK, collision, trajectory) in forked PyBullet  
- **Sampling‐based & hybrid motion planning** (straight‐line + RRT‑Connect)  
- **Plan–Validate–Execute Loop** in PyBullet GUI  

---

## 📂 Repository Layout

```

.
├── domain.pddl                 # PDDL domain (pick/place/move actions)
├── stream.pddl                 # PDDLStream stream declarations
├── pddlstream/                 # PDDLStream fork (language & algorithms)
├── pybullet\_planning/          # pybullet‑planning fork (OMPL, IK, collision)
├── meshes/                     # URDFs, mesh assets
│   └── poppy\_ergo\_jr.pybullet.urdf
├── simulation.py               # SimulationEnvironment (wrapper around PyBullet)
├── stream.py                   # Python streams: IK, sample‐pose, plan‐free, plan‐hold
├── run.py                      # `main()` → register streams, build PDDLProblem, solve, execute
├── run\_mini.py                 # simpler “mini” pick‐and‐place demo (evaluation\_simple.py)
├── submission.py               # Controller abstraction (plan/validate/execute loop)
├── evaluation.py               # 20‑block random stacking evaluation harness
├── evaluation\_simple.py        # 1‑block, 1‑swap simplified evaluation
├── example.py                  # minimal example of `Controller.run(...)`
├── test\_streams.py             # unit tests for IK & collision streams
├── test\_motion.py              # unit tests for straight‐line & RRT planning
├── requirements.txt            # `pip install -r requirements.txt`
└── README.md                   # this file

````

---

## ⚙️ Installation & Setup

1. **Clone**  
   ```bash
   git clone https://github.com/dongdongbh/s25rmp.git
   cd s25rmp
2. **Create Python environment**

   ```bash
   conda create -n s25rmp python=3.8 -y
   conda activate s25rmp
   pip install -r requirements.txt
   ```

3. **Expose PDDLStream & pybullet\_planning**

   ```bash
   export PYTHONPATH=$PYTHONPATH:$(pwd)/pddlstream:$(pwd)/pybullet_planning
   ```
---

## 🚀 Running the Full Pipeline

```bash
python run.py
```

1. **PyBullet GUI** will open with the Poppy Ergo Jr. arm loaded.
2. You’ll be prompted to add a block and sample an initial pose/grasp.
3. PDDLStream solves a pick‐and‐hold plan.
4. The plan is replayed in the GUI: pick, lift, hold.

---

## 🏃‍♂️ Other Entry Points

* **Single‐block mini demo**

  ```bash
  python run_mini.py
  ```

  Uses `evaluation_simple.py` interface to pick & place one block.

* **Batch evaluation**

  ```bash
  python evaluation.py
  ```

  Runs 20 random‐stack trials, prints accuracy, location & rotation errors.

* **Minimal Controller example**

  ```bash
  python example.py
  ```

* **Unit tests**

  ```bash
  pytest test_streams.py test_motion.py
  ```

---

## 🔧 Key APIs

| Feature                | File                             | Entry Point / API                                                      |
| ---------------------- | -------------------------------- | ---------------------------------------------------------------------- |
| Symbolic Planner       | `domain.pddl`, `run.py`          | `plan = solve(problem, ...)`                                           |
| Streams (IK / Pose /…) | `stream.py`                      | `sample_pose`, `sample_grasp`, `inverse_kin`, `plan_free`, `plan_hold` |
| Motion Planner         | `pybullet_planning/.../utils.py` | `plan_joint_motion` (RRT‑Connect)                                      |
| Simulation Env         | `simulation.py`                  | `SimulationEnvironment(show=True)`                                     |
| Controller Loop        | `submission.py`                  | `Controller.run(real_env, goal_poses)`                                 |

---

## 📖 Reproducibility

All code, PDDL files, and tests are included in this repo. The `README.md` and inline docstrings guide you through:

1. **Environment setup**
2. **Installing dependencies**
3. **Running demos, tests, and evaluations**


