(define (domain simple-kuka-pick)
  (:requirements :strips :equality)

  (:predicates
    ;; *discrete* fluents
    (Pose        ?o ?p)       ; object → pose
    (AtPose      ?o ?p)       ; at that pose
    (Grasp       ?o ?g)       ; a grasp parameter
    (AtGrasp     ?o ?g)       ; holding that grasp
    (Holding     ?o)          ; object is in gripper
    (HandEmpty)               ; gripper is empty
    (AtConf      ?q)          ; robot at configuration
    (Kin         ?o ?p ?g ?q ?t) ; a reachable ik solution
    (FreeMotion  ?q1 ?t ?q2)  ; collision‑free move (hand empty)
    (HoldingMotion ?q1 ?t ?q2 ?o ?g) ; collision‑free move (holding)

    ;; simple flags for streaming
    (Poseable  ?o)
    (Graspable ?o)
  )

  ;; ────────────────────────────────────────────────────────────────────────
  (:action move-free
    :parameters (?q1 ?t ?q2)
    :precondition (and
      (AtConf       ?q1)
      (HandEmpty)
      (FreeMotion   ?q1 ?t ?q2)
    )
    :effect (and
      (AtConf       ?q2)
      (not (AtConf ?q1))
    )
  )

  (:action move-hold
    :parameters (?q1 ?t ?q2 ?o ?g)
    :precondition (and
      (AtConf        ?q1)
      (Holding       ?o)
      (AtGrasp       ?o ?g)
      (HoldingMotion ?q1 ?t ?q2 ?o ?g)
    )
    :effect (and
      (AtConf        ?q2)
      (not (AtConf ?q1))
    )
  )

  (:action pick
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and
      (Pose        ?o ?p)
      (AtPose      ?o ?p)
      (HandEmpty)
      (AtConf      ?q)
      (Grasp       ?o ?g)
      (Kin         ?o ?p ?g ?q ?t)
    )
    :effect (and
      (AtGrasp     ?o ?g)
      (Holding     ?o)
      (not (AtPose ?o ?p))
      (not (HandEmpty))
    )
  )

  (:action place
    :parameters (?o ?p ?g ?q ?t)
    :precondition (and
      (Holding      ?o)
      (AtConf       ?q)
      (AtGrasp      ?o ?g)
      (Kin          ?o ?p ?g ?q ?t)
    )
    :effect (and
      (AtPose       ?o ?p)
      (not (Holding ?o))
      (HandEmpty)
      (not (AtGrasp   ?o ?g))
    )
  )
)

