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
        self.road_jam_control = {'RL': trafficInterface.TO_FLUID,
                                 'LR': trafficInterface.TO_FLUID,
                                 'TB': trafficInterface.TO_FLUID,
                                 'BT': trafficInterface.TO_FLUID}
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
        if 'RL' in jam_control and jam_control['RL'] is not None:
            self.road_jam_control['RL'] = jam_control['RL']

        if 'LR' in jam_control and jam_control['LR'] is not None:
            self.road_jam_control['LR'] = jam_control['LR']

        if 'TB' in jam_control and jam_control['TB'] is not None:
            self.road_jam_control['TB'] = jam_control['TB']
        if 'BT' in jam_control and jam_control['BT'] is not None:
            self.road_jam_control['BT'] = jam_control['BT']
        
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
            return {'RL': trafficInterface.TO_JAM, 'LR': trafficInterface.TO_JAM,
                    'TB': trafficInterface.TO_JAM, 'BT': trafficInterface.TO_JAM}
        elif (self.state is H_YELLOW):
            return {'RL': trafficInterface.TO_JAM, 'LR': trafficInterface.TO_JAM,
                    'TB': self.road_jam_control['TB'], 'BT': self.road_jam_control['BT']}
        elif (self.state is V_YELLOW):
            return {'RL': self.road_jam_control['RL'], 'LR': self.road_jam_control['LR'],
                    'TB': trafficInterface.TO_JAM, 'BT': trafficInterface.TO_JAM}
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
        self.IN_NEXT_JAM_LR = self.addInPort("in_jam_control_lr")
        self.OUT_JAM_LR = self.addOutPort("out_jam_control_lr")
        self.IN_NEXT_JAM_RL= self.addInPort("in_jam_control_rl")
        self.OUT_JAM_RL = self.addOutPort("out_jam_control_rl")

        self.IN_NEXT_JAM_TB = self.addInPort("in_jam_control_tb")
        self.OUT_JAM_TB = self.addOutPort("out_jam_control_tb")
        self.IN_NEXT_JAM_BT= self.addInPort("in_jam_control_bt")
        self.OUT_JAM_BT = self.addOutPort("out_jam_control_bt")

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
            jam_control['LR'] = inputs[self.IN_NEXT_JAM_LR]
            # print("pedCrossing extTransition | Jam control: ", jam_control) # debug print
        if self.IN_NEXT_JAM_RL in inputs:
            jam_control['RL'] = inputs[self.IN_NEXT_JAM_RL]
            # print("pedCrossing extTransition | Jam control: ", jam_control) # debug print
        if self.IN_NEXT_JAM_TB in inputs:
            jam_control['TB'] = inputs[self.IN_NEXT_JAM_TB]
        if self.IN_NEXT_JAM_BT in inputs:
            jam_control['BT'] = inputs[self.IN_NEXT_JAM_BT]
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
            if 'TB' in control and control['TB'] is not None:
                rv[self.OUT_JAM_TB] = control['TB']
            if 'BT' in control and control['BT'] is not None:
                rv[self.OUT_JAM_BT] = control['BT']
        return rv
    
    
