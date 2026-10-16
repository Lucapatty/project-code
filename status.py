
class States(object):
    WAITING = "waiting"
    HEATING = "heating"


class Status(object):
    
    def __init__(self):
        
        self.current_state = States.WAITING
        self.button_state = False

        return
    

    def update_sensors(self):
        # Read sensors here
        return
