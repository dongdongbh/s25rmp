(define (stream block_stack_streams)

  ;; Inverse kinematics: needs world, block, location, and grasp token
  (:stream ik
    :inputs    (?w ?b ?l ?g)
    :domain    (and (World ?w) (Location ?l))
    :outputs   (?q)
    :certified (Kin ?b ?l ?q)
  )

  ;; Collision‑free config check: needs world and a config
  (:stream cfree_config
   :inputs    (?w ?q)
   :domain    (and (World ?w))    ; no ?b, no ?l
   :outputs   ()
   :certified (CFreeConf ?q)
  )


  ;; Motion interpolation: needs world + two configs
  (:stream motion
    :inputs    (?w ?q1 ?q2)
    :domain    (and (World ?w) (CFreeConf ?q1) (CFreeConf ?q2))
    :outputs   (?t)
    :certified (Motion ?q1 ?t ?q2)
  )

  ;; Trajectory‐free: needs world, the same two configs, a trajectory, and which block
  (:stream traj_free
    :inputs    (?w ?q1 ?t ?q2 ?b)
    :domain    (and (World ?w)
                    (Motion ?q1 ?t ?q2)
                    (Holding ?b))
    :outputs   ()
    :certified (CFreeTraj ?t ?b)
  )

  ;; Generate new locations: needs world and a base
  (:stream gen-loc
    :inputs    (?w ?base)
    :domain    (and (World ?w) (Base ?base))
    :outputs   (?level ?loc)
    :certified
     (and
       (Location ?loc)
       (Clear    ?loc)
     )
  )

)

