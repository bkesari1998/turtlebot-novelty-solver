(define (problem delivery) (:domain coffee_bot)
(:objects 
    door_1 - door
    lab kitchen - room
    charger_1 - charger
    sink_1 - sink
    kitchen_wall lab_wall - wall
)

(:init
    (connect lab kitchen door_1)
    (connect kitchen lab door_1)
    (at kitchen)
    (not(open door_1))
    (inside lab charger_1)
    (inside kitchen sink_1)
    (inside lab door_1)
    (inside kitchen door_1)
    ; (inside lab lab_wall)
    ; (inside kitchen kitchen_wall)
    ; (docked)
    (facing sink_1)
)

(:goal (and
    ; (facing sink_1)
    ; (not (docked))
    ; (facing door_1)
    ; (facing kitchen_wall)
    ; (at kitchen)
    (docked)
    ; (facing door_1)
    ; (facing lab_wall)
    ; (at lab)
))
)
