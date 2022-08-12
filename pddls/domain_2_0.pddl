(define (domain navbot)

;remove requirements that are not needed
(:requirements :strips :fluents :typing :negative-preconditions)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    doorway room charger desk nothing - object
    doorway_1 - doorway
    hallway lab - room
    charger_1 - charger
    desk_1 - desk
)

; un-comment following line if constants are needed
;(:constants )
(:predicates
    (at ?r - room)
    (connect ?r1 - room ?r2 - room ?d - doorway)
    (facing ?v0 - object)
    (inside ?r1 - room ?o - object)
    (docked)
)

(:action approach
    :parameters (?object01 - object ?object02 - object ?room01 - room)
    :precondition (and
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


(:action pass_through_door
    :parameters (?room01 - room ?room02 - room ?doorway01 - doorway)
    :precondition (and
        (at ?room01)
        (connect ?room01 ?room02 ?doorway01)
        (facing ?doorway01)
     )
    :effect (and
        (at ?room02)
        (facing nothing)
     )
)

(:action dock
    :parameters (?charger01 - charger ?room01 - room )
    :precondition (and 
        (facing ?charger01)
        (at ?room01)
        (inside ?room01 ?charger01)
        (not (docked))
     )
    :effect (and
       (docked)
       (not (facing ?charger01))
       (facing nothing)
     )
)

(:action undock
    :parameters (?charger01 - charger ?room01 - room)
    :precondition (and 
        (docked)
        (inside ?room01 ?charger01)
        (at ?room01)
        (facing nothing)
     )
    :effect (and
        (at ?room01)
        (facing ?charger01)
        (not (docked))
     )
)
)