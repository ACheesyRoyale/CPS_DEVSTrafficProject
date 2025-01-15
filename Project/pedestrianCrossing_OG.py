from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

import trafficInterface
from trafficInterface import FLUID, PEDESTRIAN, UPDATE

import numpy as np

class PedestrianCrossingState:
    """Pedestrian crossing state"""
    def __init__(self, number_of_pedestrians, lamb_da, mu):
        self.state = FLUID
        self.road_jam_control = {'RL': trafficInterface.TO_FLUID,
                                 'LR': trafficInterface.TO_FLUID}
        self.number_of_pedestrians = number_of_pedestrians
        self.lamb_da = lamb_da
        self.mu = mu
        self.next_ped_time = 0.0
        self.crossing_time = 0.0
        self.prev_state = FLUID
        self.remaining_state_time = 0.0
        

    def __str__(self):
        return "self.state: " + str(self.state) + " self.road_jam_control: " + str(self.road_jam_control)

   # Calculates time for next transition 
   # Call caused by state transition (int. or ext.). 
   # = > timeAdvance = time between next state and next-next state
    def calculate_time_advance(self):
        if self.number_of_pedestrians <= 0:
            return INFINITY
        
        # UPDATE state is immediate transition
        if self.state is UPDATE:
            return 0

        # if prev_state was UPDATE, use remaining time of prev_state
        if self.prev_state is UPDATE:
            return self.remaining_state_time

        state = self.state
        self.remaining_state_time = {FLUID: self.calculate_next_pedestrian_time(),
                PEDESTRIAN: self.calculate_pedestrian_crossing_time(),
                }[state]

        return self.remaining_state_time
    
    def internal(self):
        state = self.state
        print("pedCrossState internal | state: " + str(state))

        if state is PEDESTRIAN:
            self.number_of_pedestrians -= 1
            
        self.state = {FLUID: PEDESTRIAN,
                    PEDESTRIAN: FLUID,
                    UPDATE: self.prev_state
                    }[state]
        
        self.prev_state = state
        
    def calculate_next_pedestrian_time(self):
        num_ped_per_10_min = np.random.poisson(self.lamb_da)
        # x ped per 10 - > 10 / x min between ped - > 10 / x * 60 sec between ped
        minutes_next_ped = 10 / num_ped_per_10_min
        self.next_ped_time = minutes_next_ped * 60
        return self.next_ped_time
    
    def calculate_pedestrian_crossing_time(self):
        crossing_speed = np.random.lognormal(self.mu)
        width_of_lane = 3.7
        width_of_street = 2 * width_of_lane
        self.crossing_time = width_of_street / crossing_speed
        return self.crossing_time
    
    def update_internal_jam_control(self, jam_control):
        if 'RL' in jam_control and jam_control['RL'] is not None:
            self.road_jam_control['RL'] = jam_control['RL']

        if 'LR' in jam_control and jam_control['LR'] is not None:
            self.road_jam_control['LR'] = jam_control['LR']
        
    def relay_update_to_roads(self, elapsed):
        self.remaining_state_time -= elapsed
        print("pedCros solve_jam_control | elapsed " + str(elapsed) + ", rem. state. t " + str(self.remaining_state_time))
        self.prev_state = self.state
        self.state = UPDATE

    # the output is called in the previous state.
    # when the state is pedestrian, the next state is fluid, and we should copy the state of the road ahead
    # else, the next state is pedestrian and we should jam
    def output_control(self):
        if self.state is PEDESTRIAN or self.state is UPDATE:
            return self.road_jam_control
        
        if self.state is FLUID:
            return {'RL': trafficInterface.TO_JAM, 'LR': trafficInterface.TO_JAM}


class PedestrianCrossingModel(AtomicDEVS):
    """
    Constructor for a pedestrian crossing
    :param num_pedestrians: total of pedestrians crossings to occur
    :param lamb_da: average crossings per 10 minutes
    :param mu: average crossing speed in m/s
    :return: none 
    """
    def __init__(self, num_pedestrians, lamb_da, mu):
        AtomicDEVS.__init__(self, "PedestrianCrossing")
        self.IN_NEXT_JAM_LR = self.addInPort("in_jam_control_lr")
        self.OUT_JAM_LR = self.addOutPort("out_jam_control_lr")
        self.IN_NEXT_JAM_RL= self.addInPort("in_jam_control_rl")
        self.OUT_JAM_RL = self.addOutPort("out_jam_control_rl")
        self.state = PedestrianCrossingState(num_pedestrians, lamb_da = lamb_da, mu = mu)

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
    
    
