
waypoints = {
    "lab_door_lab": [[-0.484, -3.657, 0], [ 0, 0, -0.637, 0.771]],
    "lab_door_kitchen": [[0.114, -5.891, 0], [0, 0, 7.932, 0.610]],
    "desk_refill": [[-2.130, -8.615, 0], [0, 0, 0.769, 0.639]],
    "dock_approach": [[-0.570, 1.119, 0], [ 0, 0, 0, 1]]
}

state_check = {
    "at_lab_door_lab": {"tag": "at3", "distance": 2},
    "at_dock_approach": {"tag": "at0", "distance": 2},
    "at_desk_refill": {"tag": "at9", "distance": 1},
    "lab_door_lab_open": {"tag": "at4", "in_view": True},
    "lab_door_kitchen_open": {"tag": "at13", "in_view": False},
}