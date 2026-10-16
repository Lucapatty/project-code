from status import States, Status
import state_actions as sa


class StateMachine(object):

    def __init__(self):
        self.status = Status()

        self.state_actions = {
            States.WAITING: sa.Waiting(self.status),
            States.HEATING: sa.Heating(self.status)}
        return    


    def step(self):
        """
        To be executed every time step
        """

        # Read from all sensors
        self.status.update_sensors()

        # Check all guards
        for state, action in self.state_actions.items():
            if action.guard():
                
                # Perform exit action of previous state
                self.state_actions[self.status.current_state].exit()

                # Perform transition
                self.status.current_state = state

                # Perform entry action of new state
                action.entry()

                break

        # Execute the action of the current state
        self.state_actions[self.status.current_state].run()

        return
    