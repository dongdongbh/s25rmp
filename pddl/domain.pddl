(define (domain block_stack)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    block      ; each cube
    location   ; table spots & stacking levels
    base       ; stack‐bases
    config     ; robot joint configurations
    traj       ; robot‐space trajectories
    world
  )

  (:constants
    nil    - block
    w0     - world
    baseA baseB baseC baseD baseE baseF baseG baseH  - base
  )

  (:predicates
    ;; discrete state
    (At       ?b - block    ?l - location)
    (Clear    ?l - location)
    (Empty)
    (Holding  ?b - block)
    ;; continuous‐motion predicates (certified by streams)
    (Kin        ?b - block   ?l - location ?q - config)
    (CFreeConf  ?q - config)
    (Motion     ?q1 - config ?t - traj     ?q2 - config)
    (CFreeTraj  ?t - traj     ?b - block)
    ;; stack‐bases & generated locations
    (Base      ?B - base)
    (Location  ?l - location)
  )

  (:action pick
    :parameters (?b - block ?l - location ?q - config)
    :precondition (and
      (At       ?b ?l)
      (Empty)
      (Kin      ?b ?l ?q)
      (CFreeConf ?q)
    )
    :effect (and
      (not (At       ?b ?l))
      (not (Empty))
      (Holding     ?b)
      (Clear       ?l)
    )
  )

  (:action move
    :parameters (?q1 - config ?t - traj ?q2 - config)
    :precondition (and
      (Empty)
      (Motion    ?q1 ?t ?q2)
      (CFreeTraj ?t nil)
      (CFreeConf ?q2)
    )
    :effect (and)
  )

  (:action place
    :parameters (?b - block ?l - location ?q - config)
    :precondition (and
      (Holding    ?b)
      (Clear      ?l)
      (Kin        ?b ?l ?q)
      (CFreeConf  ?q)
    )
    :effect (and
      (At       ?b ?l)
      (not (Clear    ?l))
      (Empty)
      (not (Holding ?b))
    )
  )
)

