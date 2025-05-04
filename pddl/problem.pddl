(define (problem test-block-stack)
  (:domain block_stack)

  (:objects
    b0
      - block
    baseA baseB baseC baseD baseE baseF baseG baseH
      - base
    l0 l1
      baseA_loc0 baseB_loc0 baseC_loc0 baseD_loc0
      baseE_loc0 baseF_loc0 baseG_loc0 baseH_loc0
      - location
    ;;q_home q1
     ;; - config
    ;;t_dummy
     ;; - traj
  )

  (:init
    ;; stack‐bases (declared as constants in domain)
    (Base baseA) (Base baseB) (Base baseC) (Base baseD)
    (Base baseE) (Base baseF) (Base baseG) (Base baseH)

    ;; seed each with its “level‑0” spot
    (Location baseA_loc0) (Location baseB_loc0)
    (Location baseC_loc0) (Location baseD_loc0)
    (Location baseE_loc0) (Location baseF_loc0)
    (Location baseG_loc0) (Location baseH_loc0)

    ;; your test block starts on table location l0
    (At       b0 l0)
    (Clear    l1)

    ;; declare the two table locations
    (Location l0)
    (Location l1)

    ;; gripper empty
    (Empty)

    ;; known IK: pick from l0, place at l1
    ;;(Kin      b0 l0 q_home)
    ;;(Kin      b0 l1 q1)

    ;; both configs collision‑free
    ;;(CFreeConf q_home)
    ;;(CFreeConf q1)

    ;; known free‑space motion
    ;;(Motion     q_home t_dummy q1)
    ;;(CFreeTraj  t_dummy nil)
  )

  (:goal
    (and (At b0 l1))
  )
)

