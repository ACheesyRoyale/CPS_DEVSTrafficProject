from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

import trafficInterface

GREEN = 0
YELLOW = 1
RED = 2

class LightCrossingState:
    """Pedestrian crossing state"""
    def __init__(self, time_green, time_yellow, time_red):
        self.state = GREEN
        self.road_jam_control = {'RL': trafficInterface.TO_FLUID,
                                 'LR': trafficInterface.TO_FLUID}
        self.relay_jam_control = 0
        self.t_green = time_green
        self.t_yellow = time_yellow
        self.t_red = time_red

        self.t_state = 0
        self.t_remaining = 0

        

    def __str__(self):
        return "self.state: " + str(self.state) + " self.road_jam_control: " + str(self.road_jam_control)

   # Calculates time for next transition 
   # Call caused by state transition (int. or ext.). 
    def calculate_time_advance(self):
        # relay mode is immediate action
        if self.relay_jam_control:
            return 0
        
        # Ext trans happened : 
        if self.t_remaining < self.t_state and self.t_remaining >= 0:
            self.t_state = self.t_remaining
        else:
            if self.state is GREEN:
                self.t_state = self.t_green
            elif self.state is YELLOW:
                self.t_state = self.t_yellow
            elif self.state is RED:
                self.t_state = self.t_red
            else:
                print('invalid state in time advance')

            self.t_remaining = self.t_state
        return self.t_state
    
    # internal state changes:
    def internal(self):
        # in relay mode
        if self.relay_jam_control:
            self.relay_jam_control = 0
        
        else:
            if self.state is GREEN:
                self.state = YELLOW
            elif self.state is YELLOW:
                self.state = RED
            elif self.state is RED:
                self.state = GREEN
            else:
                print('invalid state in internal')
    
    def update_internal_jam_control(self, jam_control):
        if 'RL' in jam_control and jam_control['RL'] is not None:
            self.road_jam_control['RL'] = jam_control['RL']

        if 'LR' in jam_control and jam_control['LR'] is not None:
            self.road_jam_control['LR'] = jam_control['LR']
        
    def relay_update_to_roads(self, elapsed):
        self.t_remaining -= elapsed
        self.relay_jam_control = 1


    # the output is called right before the internal change.
    # TO_JAM if going to Yellow or Red
    # else copy road ahead, going to Green or Relay
    def output_control(self):
        if (self.state is GREEN) or (self.state is YELLOW):
            return {'RL': trafficInterface.TO_JAM, 'LR': trafficInterface.TO_JAM}
        else:
            return self.road_jam_control
            

class LightCrossingModel(AtomicDEVS):
    '''
    Constructor for a pedestrian crossing

    Attributes:
    :param num_pedestrians: total of pedestrians crossings to occur
    :param lamb_da: average crossings per 10 minutes
    :param mu: average crossing speed in m/s
    :return: none 
    '''
    def __init__(self, t_green, t_yellow, t_red):
        AtomicDEVS.__init__(self, "TrafficLightCrossing")
        self.IN_NEXT_JAM_LR = self.addInPort("in_jam_control_lr")
        self.OUT_JAM_LR = self.addOutPort("out_jam_control_lr")
        self.IN_NEXT_JAM_RL= self.addInPort("in_jam_control_rl")
        self.OUT_JAM_RL = self.addOutPort("out_jam_control_rl")
        self.state = LightCrossingState(t_green, t_yellow, t_red)

    # defines the next state for each state
    # This is the only function allowed to change the state.
    def intTransition(self):
        self.state.internal()
        return self.state

    def __str__(self):
        return super().__str__()
    
    # Receives JAM_Control from next section
    def extTransition(self, inputs):
        jam_control = {}
        if self.IN_NEXT_JAM_LR in inputs:
            jam_control['LR'] = inputs[self.IN_NEXT_JAM_LR]
            # print("pedCrossing extTransition | Jam control: ", jam_control) # debug print
        if self.IN_NEXT_JAM_RL in inputs:
            jam_control['RL'] = inputs[self.IN_NEXT_JAM_RL]
            # print("pedCrossing extTransition | Jam control: ", jam_control) # debug print
        if jam_control != {}:
            self.state.update_internal_jam_control(jam_control)
            self.state.relay_update_to_roads(self.elapsed)
        return self.state
    
    # defines the delay in each state
    # self.state is read-only
    def timeAdvance(self):
        return self.state.calculate_time_advance()


    # is called right before the internal change
    def outputFnc(self):
        control = self.state.output_control()
        rv= {}
        if control is not None:
            if 'LR' in control and control['LR'] is not None:
                rv[self.OUT_JAM_LR] = control['LR']
            if 'RL' in control and control['RL'] is not None:
                rv[self.OUT_JAM_RL] = control['RL']
        return rv
    
    
