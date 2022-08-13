(define (problem problem_0) (:domain coffee_bot)
(:objects
	nothing - nothing
	charger_1 - charger
	doorway_1 - doorway
	lab hallway - room
	desk_1 - desk
)

(:init
	(facing ['charger_1'])
	(docked)
	(at ['lab'])
	(inside lab hallway desk_1)
	(connect lab hallway desk_1)
	(inside lab hallway desk_1)
	(inside hallway desk_1)
	(inside lab desk_1)
)

(:goal (and
	(facing desk_1)
))
)