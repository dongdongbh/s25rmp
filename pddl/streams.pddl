(define (stream block_stack_streams)
  ;; Inverse kinematics for pick & place:
  (:stream ik
    :inputs    (?w ?b ?l ?g)
    :domain (and (World ?w) (Block ?b) (Location ?l) (Grasp ?g))
    :outputs   (?q)
    :certified (Kin ?b ?l ?q)
  )

  ;; Collision-free config check
  (:stream cfree_config
    :inputs    (?w ?q)
    :domain    (Config ?q)
    :certified (CFreeConf ?q)
  )

  ;; Motion interpolation (with RRT fallback)  
  (:stream motion
    :inputs    (?w ?q1 ?q2)
    :domain    (and (Config ?q1) (Config ?q2))
    :outputs   (?t)
    :certified (Motion ?q1 ?t ?q2)
  )

  ;; Collision-free trajectory when carrying ?b
  (:stream traj_free
   :inputs    (?w ?t ?b)
   :domain    (and (Holding ?b))
   :certified (CFreeTraj ?t ?b)
  )

  ;; Dynamically generate new stacking locations
  (:stream gen-loc
    :inputs    (?w ?base)
    :domain    (Base ?base)
    :outputs   (?level ?loc)
    :certified (Location ?loc)
  )
)

