(define (domain block_stack)
  (:requirements :strips :negative-preconditions)

  ;; no :types section — all arguments are untyped

  (:constants
    nil w0 baseA baseB baseC baseD baseE baseF baseG baseH
  )

  (:predicates
    ;; discrete state
    (At       ?b ?l)
    (Clear    ?l)
    (Empty)
    (Holding  ?b)

    ;; continuous‐motion predicates (certified by streams)
    (Kin       ?b ?l ?q)
    (CFreeConf ?q)
    (Motion    ?q1 ?t ?q2)
    (CFreeTraj ?t ?b)

    ;; stack‐bases & generated locations
    (Base     ?B)
    (Location ?l)
  )

  (:action pick
    :parameters (?b ?l ?q)
    :precondition (and
      (At        ?b ?l)
      (Empty)
      (Kin       ?b ?l ?q)
      (CFreeConf ?q)
    )
    :effect (and
      (not (At        ?b ?l))
      (not (Empty))
      (Holding     ?b)
      (Clear       ?l)
    )
  )

  (:action move
    :parameters (?q1 ?t ?q2)
    :precondition (and
      (Empty)
      (Motion     ?q1 ?t ?q2)
      (CFreeTraj  ?t nil)
      (CFreeConf  ?q2)
    )
    :effect (and)
  )

  (:action place
    :parameters (?b ?l ?q)
    :precondition (and
      (Holding    ?b)
      (Clear      ?l)
      (Kin        ?b ?l ?q)
      (CFreeConf  ?q)
    )
    :effect (and
      (At       ?b ?l)
      (not (Clear      ?l))
      (Empty)
      (not (Holding ?b))
    )
  )
)

