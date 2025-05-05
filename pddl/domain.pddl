(define (domain block_stack)
    (:requirements 
    :strips 
    :typing 
    :negative-preconditions 
    :existential-preconditions   
    :derived-predicates          
  )


  (:constants
    nil w0 baseA baseB baseC baseD baseE baseF baseG baseH
  )

  (:predicates
    ;; discrete state
    (At       ?b ?l)
    (Clear    ?l)
    (Holding  ?b)
    (World    ?w - world)
    (Supported ?l)
    (OnFloor   ?l)
    (Above ?lower ?upper)
    (Config ?q - config)
    (Empty)
    (Block   ?b - block)


    ;; continuous‐motion predicates (certified by streams)
    (Kin       ?b ?l ?q)
    (CFreeConf ?q)
    (Motion    ?q1 ?t ?q2)
    (CFreeTraj ?t ?b)

    ;; stack‐bases & generated locations
    (Base     ?B)
    (Location ?l)
  )
  ;;----------------------------------------
  ;; Axiom: Supported if OnFloor or if a block is AT in a spot below
  ;;----------------------------------------
  (:derived (Supported ?l)
   (or
    (OnFloor ?l)
    (exists (?lower - location
             ?b     - block)
     (and
      (Above ?lower ?l)
      (At    ?b      ?lower)
      (not (= ?b nil))
     )
    )
   )
  )

  (:action pick
    :parameters (?b ?l ?q)
    :precondition (and
        (Config ?q)
      (Holding  nil)
      (not (= ?b nil))
      (At        ?b ?l)
      (Location   ?l)
      (Kin       ?b ?l ?q)
      (CFreeConf ?q)
    )
    :effect (and
      (not (At        ?b ?l))
      (not (Holding  nil))
      (Holding     ?b)
      (Clear       ?l)
    )
  )

  (:action move
    :parameters (?b ?q1 ?t ?q2)
    :precondition (and
      (Holding    ?b)
        (Config ?q1)
      (Motion     ?q1 ?t ?q2)
      (CFreeTraj  ?t nil)
      (CFreeConf  ?q2)
    )
    :effect (and
(not (Config ?q1))
      (Config ?q2)
)
  )

  (:action place
    :parameters (?b ?l ?q)
    :precondition (and
        (Config ?q)
      (Holding    ?b)
      (not (= ?b nil))
      (Location   ?l)
      (Clear      ?l)
      (Supported  ?l)
      (Kin        ?b ?l ?q)
      (CFreeConf  ?q)
    )
    :effect (and
      (At       ?b ?l)
      (not (Clear      ?l))
      (not (Holding ?b))
      (Holding  nil)
    )
  )
)
