# test_extract_physical_world.py

import pybullet as pb
from simulation import SimulationEnvironment
from planners.pddlstream_interface import extract_physical_world, _SHARED_WORLD

def main():
    print("=== Starting extract_physical_world test ===")
    env = SimulationEnvironment(show=False)
    try:
        # 1) Add a dynamic block and settle
        label1 = env._add_block((0.1, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0), mass=1, side=0.02)
        env.settle(0.5)

        # 2) Populate _SHARED_WORLD with a dummy entry for label1
        #    (simulating the planner having seen it before)
        _SHARED_WORLD[label1] = ((9.9, 9.9, 9.9), (1.0, 0.0, 0.0, 0.0))

        # 3) Normal extraction: should succeed for both b0 (platform) and label1
        print("\n-- Normal extraction --")
        state = extract_physical_world(env)
        print("extract_physical_world returned:")
        for b, pose in state.items():
            print(f"  {b} -> {pose}")
        assert 'b0' in state, "Platform block 'b0' should be present"
        assert label1 in state, f"Dynamic block '{label1}' should be present"

        # 4) Monkey‐patch get_block_pose to force a failure on label1
        orig_get = env.get_block_pose
        def broken_get_block_pose(lbl):
            if lbl == label1:
                raise pb.error("forced failure")
            return orig_get(lbl)
        env.get_block_pose = broken_get_block_pose

        # 5) Fallback extraction: label1’s pose should come from _SHARED_WORLD
        print("\n-- Forced‐failure extraction (label1 fails) --")
        state2 = extract_physical_world(env)
        print("extract_physical_world returned:")
        for b, pose in state2.items():
            print(f"  {b} -> {pose}")
        assert label1 in state2, "Even on get_block_pose failure, label1 must appear"
        assert state2[label1] == _SHARED_WORLD[label1], (
            "On failure, label1’s pose should fall back to _SHARED_WORLD"
        )

        print("\n✅ extract_physical_world passed both normal and fallback cases!")

    finally:
        env.close()

if __name__ == "__main__":
    main()

