(define (domain eval_1)
(:requirements :strips :typing)
(:constants air)
(:predicates (on ?place ?cube))
(:action pickplace
:parameters (?place_i ?place_k ?_cube)
:precondition (and (on ?place_k air) (on ?place_i ?_cube))
:effect (and (on ?place_i air) (on ?place_k ?_cube) (not (on ?place_k air)) (not (on ?place_i ?_cube)))))


