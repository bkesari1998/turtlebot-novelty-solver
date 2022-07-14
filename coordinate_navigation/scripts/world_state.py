agents = {
    "turtlebot": {"at": "lab", "facing": "charger_1", "docked": True}
}

rooms = {
    "lab": {},
    "kitchen": {},
    "office": {} 
}

doors = {
    "lab_door": {"connect": ["lab", "kitchen"], "open": False},
    "office_door": {"connect": ["office", "kitchen"], "open": False}
}

chargers = {
    "charger_1": {"inside": "lab"},
    "charger_2": {"inside": "lab"}
}

desks = {
    "desk_refil": {"inside": "kitchen"}
}

