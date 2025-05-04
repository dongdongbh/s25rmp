(define (stream block_stack_streams)

  ;; Inverse kinematics: needs world, block, location
  (:stream ik
    :inputs  (?w ?b ?l)
    :domain  (and
               (World    ?w)
               (Block    ?b)
               (Location ?l)
             )
    :outputs (?q)
    :certified (Kin ?b ?l ?q)
  )

  ;; Collision‑free config check: needs world and the config to check
  (:stream cfree_config
    :inputs  (?w ?q)
    :domain  (and
               (World  ?w)
               (Config ?q)
             )
    :outputs ()
    :certified (CFreeConf ?q)
  )

  ;; Motion interpolation: needs world + two collision‑free configs
  (:stream motion
    :inputs  (?w ?q1 ?q2)
    :domain  (and
               (World     ?w)
               (CFreeConf ?q1)
               (CFreeConf ?q2)
             )
    :outputs (?t)
    :certified (Motion ?q1 ?t ?q2)
  )

  ;; Collision‑free trajectory when carrying ?b: needs world, path, and b
  (:stream traj_free
    :inputs  (?w ?q1 ?t ?q2 ?b)
    :domain  (and
               (World     ?w)
               (Motion    ?q1 ?t ?q2)
               (Holding   ?b)
             )
    :outputs ()
    :certified (CFreeTraj ?t ?b)
  )

  ;; Dynamically generate new stacking locations: needs world and a base
  (:stream gen-loc
    :inputs  (?w ?base)
    :domain  (and
               (World ?w)
               (Base  ?base)
             )
    :outputs   (?loc)
    :certified (and
                  (Location ?loc)
                  (Clear    ?loc)
                )
  )
)

