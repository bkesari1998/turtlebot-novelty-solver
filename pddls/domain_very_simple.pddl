(define (domain coffee_bot)

;remove requirements that are not needed
(:requirements :strips :fluents :typing :negative-preconditions)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    door room charger sink wall - object
    door_1 - door
    kitchen_wall lab_wall - wall
    lab kitchen - room
    charger_1 - charger
    sink_1 - sink
)

; un-comment following line if constants are needed
;(:constants )
(:predicates
    (at ?r - room)
    (connect ?r1 - room ?r2 - room ?d - door)
    (facing ?v0 - object)
    (open ?v0 - door)
    (inside ?r1 - room ?o - object)
    (docked)
)

(:action approach
    :parameters (?object01 - object ?object02 - object ?room01 - room)
    :precondition (and
        ; (connect ?room01 ?room02 ?door01)
        (inside ?room01 ?object01)
        (inside ?room01 ?object02)
        (at ?room01)
        (facing ?object01)
        (not (docked))
     )
    :effect ( and
    (facing ?object02)
    )
)

(:action open_door
    :parameters (?door01 - door ?room01 - room ?room02 - room ?wall01 - wall)
    :precondition (and
        (not (open ?door01))
        (facing ?door01)
         )
    :effect (and 
        (open ?door01)
        (facing ?wall01)
    )
)

(:action pass_through_door
    :parameters (?room01 - room ?room02 - room ?door01 - door ?wall01 - wall)
    :precondition (and
        (at ?room01)
        (connect ?room01 ?room02 ?door01)
        (open ?door01)
        (facing ?wall01)
     )
    :effect (and
        (at ?room02)
     )
)

(:action dock
    :parameters (?room01 - room ?charger01 - charger)
    :precondition (and 
        (facing ?charger01)
        (at ?room01)
        (inside ?room01 ?charger01)
        (not (docked))
     )
    :effect (and
       (docked)
     )
)

(:action undock
    :parameters (?room01 - room ?charger01 - charger)
    :precondition (and 
        (docked)
        (inside ?room01 ?charger01)
        (at ?room01)
     )
    :effect (and
        (at ?room01)
        (facing ?charger01)
        (not (docked))
     )
)
)