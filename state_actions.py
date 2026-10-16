from status import States


class StateActionBase(object):
    """
    Template class for all state action classes.
    """

    def __init__(self, status):
        self.status = status
        return
    
    def guard(self):
        return False

    def entry(self):
        return
    
    def run(self):
        return

    def exit(self):
        return
    

class Waiting(StateActionBase):
    
    def __init__(self, status):
        super().__init__(status)
        return
    

    def guard(self):
        return (self.status.current_state == States.HEATING and
            not self.status.button_state)


    def run(self):
        print("Waiting!")
        return
    

class Heating(StateActionBase):
    
    def __init__(self, state):
        super().__init__(state)
        return


    def guard(self):
        return (self.status.current_state == States.WAITING and 
            self.status.button_state)


    def entry(self):
        print("Warning! Getting hot!")
        return


    def run(self):
        print("Heating")
        return