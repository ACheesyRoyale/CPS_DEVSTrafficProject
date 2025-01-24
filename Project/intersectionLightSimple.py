from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

import trafficInterface

H_GREEN = 0
H_YELLOW = 1
H_RED = 2
V_YELLOW = 3

class SimpleIntersectionLightState:
    """Pedestrian crossing state"""
    def __init__(self, time_green, time_yellow, time_red):
        self.state = H_GREEN
        self.road_jam_control = {'R': trafficInterface.TO_FLUID,
                                 'L': trafficInterface.TO_FLUID,
                                 'T': trafficInterface.TO_FLUID,
                                 'B': trafficInterface.TO_FLUID}
        self.relay_jam_control = 0
        self.t_hGreen = time_green
        self.t_hYellow = time_yellow
        self.t_hRed = time_red
        self.t_vYellow = time_yellow

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
            if self.state is H_GREEN:
                self.t_state = self.t_hGreen
            elif self.state is H_YELLOW:
                self.t_state = self.t_hYellow
            elif self.state is H_RED:
                self.t_state = self.t_hRed
            elif self.state is V_YELLOW:
                self.t_state = self.t_vYellow
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
            if self.state is H_GREEN:
                self.state = H_YELLOW
            elif self.state is H_YELLOW:
                self.state = H_RED
            elif self.state is H_RED:
                self.state = V_YELLOW
            elif self.state is V_YELLOW:
                self.state = H_GREEN
            else:
                print('invalid state in internal')
    
    def update_internal_jam_control(self, jam_control):
        if 'R' in jam_control and jam_control['R'] is not None:
            self.road_jam_control['L'] = jam_control['R']

        if 'L' in jam_control and jam_control['L'] is not None:
            self.road_jam_control['R'] = jam_control['L']

        if 'T' in jam_control and jam_control['T'] is not None:
            self.road_jam_control['B'] = jam_control['T']
        if 'B' in jam_control and jam_control['B'] is not None:
            self.road_jam_control['T'] = jam_control['B']
        
    def relay_update_to_roads(self, elapsed):
        self.t_remaining -= elapsed
        self.relay_jam_control = 1


    # the output is called right before the internal change.
    # if H_Green == going to H_Yellow: horizontal goes to Yellow:
    #   horizontal: to jam
    #   vertical: to jam
    # if H_Yellow == going to H_Red: horizontal goes to Red:
    #   horizontal: to jam
    #   vertical: copy road_jam
    # if H_Red == going to V_Yellow: vertical goes to Yellow:
    #   horizontal: to jam
    #   vertical: to jam
    # if V_Yellow == going to H_Green: horizontal goes to Green:
    #   horizontal: copy road_jam
    #   vertical: to jam
    # else copy road ahead, going to Green or Relay
    def output_control(self):
        if (self.state is H_GREEN) or (self.state is H_RED):
            return {'R': trafficInterface.TO_JAM, 'L': trafficInterface.TO_JAM,
                    'T': trafficInterface.TO_JAM, 'B': trafficInterface.TO_JAM}
        elif (self.state is H_YELLOW):
            return {'R': trafficInterface.TO_JAM, 'L': trafficInterface.TO_JAM,
                    'T': self.road_jam_control['T'], 'B': self.road_jam_control['B']}
        elif (self.state is V_YELLOW):
            return {'R': self.road_jam_control['R'], 'L': self.road_jam_control['L'],
                    'T': trafficInterface.TO_JAM, 'B': trafficInterface.TO_JAM}
        else:
            return self.road_jam_control
            

class SimpleIntersectionLightModel(AtomicDEVS):
    '''
    Constructor for a pedestrian crossing

    Attributes:
    :param num_pedestrians: total of pedestrians crossings to occur
    :param lamb_da: average crossings per 10 minutes
    :param mu: average crossing speed in m/s
    :return: none 
    '''
    def __init__(self, hor_t_hGreen, hor_t_hYellow, hor_t_hRed, name=""):
        AtomicDEVS.__init__(self, "IntersectionSimpleLight"+name)
        self.IN_NEXT_JAM_L = self.addInPort("in_jam_control_l")
        self.OUT_JAM_L = self.addOutPort("out_jam_control_l")
        self.IN_NEXT_JAM_R= self.addInPort("in_jam_control_r")
        self.OUT_JAM_R = self.addOutPort("out_jam_control_r")

        self.IN_NEXT_JAM_T = self.addInPort("in_jam_control_t")
        self.OUT_JAM_T = self.addOutPort("out_jam_control_t")
        self.IN_NEXT_JAM_B= self.addInPort("in_jam_control_b")
        self.OUT_JAM_B = self.addOutPort("out_jam_control_b")

        self.state = SimpleIntersectionLightState(hor_t_hGreen, hor_t_hYellow, hor_t_hRed)

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
            jam_control['R'] = inputs[self.IN_NEXT_JAM_R]
            # print("pedCrossing extTransition | Jam control: ", jam_control) # debug print
        if self.IN_NEXT_JAM_RL in inputs:
            jam_control['L'] = inputs[self.IN_NEXT_JAM_L]
            # print("pedCrossing extTransition | Jam control: ", jam_control) # debug print
        if self.IN_NEXT_JAM_TB in inputs:
            jam_control['B'] = inputs[self.IN_NEXT_JAM_B]
        if self.IN_NEXT_JAM_BT in inputs:
            jam_control['T'] = inputs[self.IN_NEXT_JAM_T]
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
            if 'R' in control and control['R'] is not None:
                rv[self.OUT_JAM_R] = control['R']
            if 'L' in control and control['L'] is not None:
                rv[self.OUT_JAM_L] = control['L']
            if 'B' in control and control['B'] is not None:
                rv[self.OUT_JAM_B] = control['B']
            if 'T' in control and control['T'] is not None:
                rv[self.OUT_JAM_T] = control['T']
        return rv
    
    
