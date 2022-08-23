(define (domain coffee_bot)

;remove requirements that are not needed
(:requirements :strips :fluents :typing :disjunctive-preconditions)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    door room charger desk - object
    lab_door office_door kitchen_door - door
    lab office hallway kitchen - room
    charger_1 charger_2 - charger
    desk_1 desk_2 desk_3 desk_4 desk_5 desk_6 desk_refill - desk
    keurig_cup sugar milk water charge - entity
)

; un-comment following line if constants are needed
;(:constants )
(:predicates
    (at ?r - room)
    (connect ?r1 - room ?r2 - room ?d - door)
    (facing ?v0 - object)
    (open ?v0 - door)
    (inside ?r1 - room ?o - object)
    ; (delivered ?d1 - desk)
    (docked)
    ; (making_coffee)
)

(:functions ;todo: define numeric functions here
   (world ?v0 - object)
;   (inventory ?v0 - entity)
;   (battery ?v0 - charge)
)

(:action approach_door 
    :parameters (?door01 - door ?room01 - room ?room02 - room)
    :precondition (and
        (connect ?room01 ?room02 ?door01)
        (at ?room01)
        (not(docked))
     )
    :effect (facing ?door01)
)

(:action open_door
    :parameters (?door01 - door ?room01 - room ?room02 - room)
    :precondition (and
        (connect?room01 ?room02 ?door01)
        (not (open ?door01))
        (facing ?door01)
         )
    :effect (and 
        (open ?door01)
        (not (facing ?door01))
    )
)

(:action exit_room ; can also be interpreted as enter_room 
    :parameters (?room01 - room ?room02 - room ?door01 - door)
    :precondition (and
        (at ?room01)
        (not(docked))
        (connect ?room01 ?room02 ?door01)
        ; (facing ?door01)
        (open ?door01)
     )
    :effect (and
        (at ?room02)
        (not (at ?room01))
        ; (not (facing ?door01))
     )
)


; (:action approach_desk
;     :parameters (?desk01 - desk ?room01 - room)
;     :precondition (and
;         (at ?room01)
;         (inside ?room01 ?desk01)
;      )
;     :effect (and 
;         (facing ?desk01)
;     )
; )


; (:action make_coffee
;     :parameters (?desk01 - desk ?room01 - room)
;     :precondition (and
;         (facing ?desk01)
;         (>= (inventory sugar) 5)
;         (>= (inventory keurig_cup) 5)
;         (>= (inventory water) 5)
;         (>= (inventory milk) 5)
;         (>= (inventory charge) 50)
;      )
;     :effect (and
;         (delivered ?desk01)
;         (making_coffee)
;      )
; )

;;;;;;;;;;;Kitchen related actions;;;;;;;;;;;;;;;;;;;;;

(:action approach_desk_refill
    :parameters (?room01 - room ?desk01 - desk)
    :precondition (and
        (not(docked))
        (at ?room01)
        (inside ?room01 ?desk01)
     )
    :effect (and
        (facing ?desk01)
        ; (increase (inventory sugar) 5)
        ; (increase (inventory keurig_cup) 2)
        ; ((increase (inventory water) 5)

     )
)

        ; (or
        ; (<=(inventory sugar) 4)
        ; (<= (inventory keurig_cup) 2)
        ; (<= (inventory water) 2)
        ; )
        
; (:action refill
;     :parameters (?desk01 - desk_refill)
;     :precondition (and
;          (facing ?desk01)
;          (or
;             (<=(inventory sugar) 4)
;             (<= (inventory keurig_cup) 2)
;             (<= (inventory water) 2)
;      )
;     )
;     :effect (and
;         (increase (inventory sugar) 5)
;         (increase (inventory keurig_cup) 2)
;         (increase (inventory water) 5)
;      )
; )

;;;;;;;;;;;;charging related actions;;;;;;;;;;;;;;;;;;;;;

; (:action approach_charger
;     :parameters (?room01 - room ?charger01 - charger)
;     :precondition (and 
;         (<=(inventory charge) 25)
;      )
;     :effect (and
;         (facing ?charger01)
;      )
; )

(:action dock
    :parameters (?room01 - room ?charger01 - charger)
    :precondition (and 
        (facing ?charger01)
        (at ?room01)
        (inside ?room01 ?charger01)
     )
    :effect (and
       (docked)
       (facing ?charger01)
     )
)

(:action undock
    :parameters (?room01 - room ?charger01 - charger)
    :precondition (and 
        (docked)
        (facing ?charger01)
        (inside ?room01 ?charger01)

     )
    :effect (and
        (facing ?charger01)
        (not (docked))
     )
)

; (:action charge
;     :parameters ()
;     :precondition (and
;         (docked)
;         (<=(battery charge) 25)
;         ; (<=(inventory charge) 25)
;      )
;     :effect (and
;         (increase (battery charge) 75)
;      )
; )

)