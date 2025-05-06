(define (stream simple-kuka-pick)
 (:stream sample-grasp
  :inputs (?o)
  :domain (Graspable ?o)
  :outputs (?g)
  :certified (Grasp ?o ?g)
 )


 (:stream sample-pose
  :inputs (?o)
  :domain (Poseable ?o)
  :outputs (?p)
  :certified (and (Pose ?o ?p) (AtPose ?o ?p))
 )

 (:stream inverse-kin
  :inputs (?o ?p ?g)
  :domain (and (Pose ?o ?p) (Grasp ?o ?g))
  :outputs (?q ?t)
  :certified (and (AtConf ?q) (Kin ?o ?p ?g ?q ?t))
 )

 (:stream plan-free
  :inputs (?q1 ?q2)
  :domain (and (AtConf ?q1) (HandEmpty) (AtConf ?q2))
  :outputs (?t)
  :certified (FreeMotion ?q1 ?t ?q2)
 )

 (:stream plan-hold
  :inputs (?q1 ?q2 ?o ?g)
  :domain (and (AtConf ?q1) (Holding ?o) (AtGrasp ?o ?g) (AtConf ?q2))
  :outputs (?t)
  :certified (HoldingMotion ?q1 ?t ?q2 ?o ?g)
 )
 )

