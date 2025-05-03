;; pddl/domain.pddl
  (define (domain block_stack)
   (:requirements :strips :typing :negative-preconditions)

   (:types
    block      ; each cube
    location   ; base positions
    base       ; stack‑bases
    config     ; robot joint configurations
    traj       ; robot‑space trajectories
    world
   )

   (:constants
    nil    - block
    w0     - world
    baseA baseB baseC baseD baseE baseF baseG baseH  - base
   )


   (:predicates
    ;; where each block lives
    (At      ?b - block   ?l - location)
    (Clear   ?l - location)            ; no block currently occupying ?l

    ;; hand‐state
    (Empty)                           ; gripper is empty
    (Holding ?b - block)              ; gripper holds block ?b

    ;; continuous‐motion predicates (to be certified by streams)
    (Kin        ?b - block ?l - location ?q - config)
    (CFreeConf  ?q - config)                       ; q is collision‐free
    (Motion     ?q1 - config ?t - traj ?q2 - config)
    (CFreeTraj  ?t - traj   ?b - block)             ; carrying ?b along t is free
    (Base  ?b - base)
    (Location ?l - location)
   )

   (:action pick
    :parameters (?b - block ?l - location ?q - config)
    :precondition  ; 
    (and
     (At         ?b ?l)
     (Empty)
     (Kin        ?b ?l ?q)
     (CFreeConf  ?q)
    )
    :effect
    (and
     (not (At       ?b ?l))
     (not (Empty))
     (Holding     ?b)
     (Clear       ?l)
    )
   )

   (:action move
    :parameters (?q1 - config ?t - traj ?q2 - config)
    :precondition
    (and
     (Empty)
     (Motion    ?q1 ?t ?q2)
     (CFreeTraj ?t nil)
     (CFreeConf ?q2)
    )
    :effect
    (and)  ; empty effect is allowed
   )

   (:action place
    :parameters (?b - block ?l - location ?q - config)
    :precondition  ; 
    (and
     (Holding   ?b)
     (Clear     ?l)
     (Kin       ?b ?l ?q)
     (CFreeConf ?q)
    )
    :effect
    (and
     (At       ?b ?l)
     (not (Clear    ?l))
     (Empty)
     (not (Holding ?b))
    )
   )
   )

