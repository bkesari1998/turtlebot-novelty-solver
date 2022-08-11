(define (problem delivery) (:domain navbot)
(:objects 
    doorway_1 - doorway
    lab hallway - room
    charger_1 - charger
    desk_1 - desk
    nothing - nothing
)

(:init
    ; always true states
    (connect lab hallway doorway_1)
    (connect hallway lab doorway_1)
    (inside lab charger_1)
    (inside hallway desk_1)
    (inside lab doorway_1)
    (inside hallway doorway_1)
    ; variable states
    (at hallway)
    (not (docked))
    (facing desk_1)
)

(:goal (and
    (docked)
))
)