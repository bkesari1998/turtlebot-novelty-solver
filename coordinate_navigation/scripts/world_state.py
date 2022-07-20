global agents
agents = {
    "turtlebot": {"at": "lab", "facing": "charger_1", "docked": True}
}

global objects
objects = {
    "room": {
        "lab": {},
        "kitchen": {},
        "office": {} 
    }, 

    "door": {
        "lab_door": {"connect": ["lab", "kitchen"], "open": False},
        "office_door": {"connect": ["office", "kitchen"], "open": False}
    },

    "charger": {
        "charger_1": {"inside": "lab"},
        "charger_2": {"inside": "lab"}
    },

    "desk": {
        "desk_refill": {"inside": "kitchen"}
    }
}
