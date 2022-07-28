
waypoints = {
    "lab_door_lab": [[21.24517, 18.35567, 0], [ 0, 0, 0.07920, 0.99686]],
    "lab_door_kitchen": [[24.12419, 19.03843, 0], [0, 0, -0.99185, 0.12741]],
    "desk_refill": [[27.28736, 17.25920, 0], [0, 0, -0.98792, 0.15495]],
    "dock_approach": [[16.12807, 19.31461, 0], [ 0, 0, 0.66022, 0.75107]]
}

state_check = {
    "at_lab_door_lab": {"tag": "at3", "distance": 2},
    "lab_door_lab_open": {"tag": "at4", "in_view": True},
    "lab_door_kitchen_open": {"tag": "at13", "in_view": False},
    "at_dock_approach": {"tag": "at0", "distance": 2},
    "at_desk_refill": {"tag": "at9", "distance": 1},
}