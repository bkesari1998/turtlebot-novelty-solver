global agents
agents = {
    "turtlebot": {"at": "lab", "facing": "charger_1", "docked": True}
}

global rooms
rooms = {
    "lab": {},
    "kitchen": {},
    "office": {} 
}

global doors
doors = {
    "lab_door": {"connect": ["lab", "kitchen"], "open": False},
    "office_door": {"connect": ["office", "kitchen"], "open": False}
}

global chargers
chargers = {
    "charger_1": {"inside": "lab"},
    "charger_2": {"inside": "lab"}
}

global desks
desks = {
    "desk_refil": {"inside": "kitchen"}
}

