(define (problem delivery) (:domain coffee_bot)
(:objects 
    lab_door office_door - door
    lab office kitchen - room
    charger_1 charger_2 - charger
    desk_1 desk_2 desk_3 desk_4 desk_5 desk_6 desk_refill - desk
)

(:init
    ; (= (inventory sugar) 0)
    ; (= (inventory keurig_cup) 0)
    ; (= (inventory water) 0)
    ; (= (inventory milk) 0)
    ; (= (world sugar) 100)
    ; (= (world keurig_cup) 100)
    ; (= (world water) 100)
    ; (= (world milk) 50)    
    ; (= (inventory charge) 10)
    (connect lab kitchen lab_door)
    (connect office kitchen office_door)
    (at lab)
    (facing charger_1) ;(facing charger_2)
    (not(open lab_door))
    ; (open office_door)
    (inside lab charger_1)
    (inside lab charger_2)
    ; (inside office desk_3)
    ; (inside office desk_4)
    ; (inside office desk_5)
    ; (inside office desk_6)
    (inside kitchen desk_refill)
    ; (inside kitchen sugar)
    ; (inside kitchen keurig_cup)
    ; (inside kitchen water)
    ; (inside kitchen milk)
    ; (not(docked))
    (docked)
    ; (not (making_coffee))
)

(:goal (and
    ; (delivered desk_2)
    ; (making_coffee)
    ; (at kitchen)
    (facing desk_refill)
))
;un-comment the following line if metric is needed
;(:metric minimize (???))
)
