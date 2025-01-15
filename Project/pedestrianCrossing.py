from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

import trafficInterface
from trafficInterface import FLUID, PEDESTRIAN, UPDATE

import numpy as np

class PedestrianCrossingState:
    """Pedestrian crossing state"""
    def __init__(self, passed_pedestrians, avgCrossingsPerTenMinutes, avgCrossingSpeed):
        self.state = 0
        self.road_jam_control = {'RL': trafficInterface.TO_FLUID,
                                 'LR': trafficInterface.TO_FLUID}
        self.passed_pedestrians = passed_pedestrians
        self.avgCrossingsPerTenMinutes = avgCrossingsPerTenMinutes
        self.avgCrossingSpeed = avgCrossingSpeed
        self.next_ped_time = 0.0
        self.crossing_time = 0.0
        self.relay_jam_control = 0
        self.state_time = 0.0
        

    def __str__(self):
        return "self.state: " + str(self.state) + " self.road_jam_control: " + str(self.road_jam_control)

   # Calculates time for next transition 
   # Call caused by state transition (int. or ext.). 
    def calculate_time_advance(self):
        # passed all pedestrians we wanted to pass
        if self.passed_pedestrians <= 0:
            return INFINITY
        
        # relay mode is immediate action
        if self.relay_jam_control:
            return 0
        
        # no pedestrian crossing - > time till next pedestrian
        if self.state == 0:
            self.next_ped_time = self.calculate_next_pedestrian_time()
            self.crossing_time = 0.0
            self.state_time = self.next_ped_time

        # pedestrian crossing - > min(nextPed, crossTime)
        if self.state > 0:
            if self.next_ped_time == 0.0:
                self.next_ped_time = self.calculate_next_pedestrian_time()
            if self.crossing_time == 0.0:
                self.crossing_time = self.calculate_pedestrian_crossing_time()
        
            self.state_time = min(self.crossing_time, self.next_ped_time)

        return self.state_time
    
    # internal state changes:
    # new ped or ped crossed
    def internal(self):
        # in relay mode
        if self.relay_jam_control:
            self.relay_jam_control = 0
            
        # no ped and new ped time != 0 - > add ped
        elif self.state == 0 and self.next_ped_time != 0.0:
            self.next_ped_time = 0.0
            self.state = 1
            self.passed_pedestrians -= 1
            
        else:
            # peds are crossing : next event?
            pedCrossed = (self.crossing_time - self.state_time == 0.0)
            newPed = (self.next_ped_time - self.state_time == 0.0)

            # newPed joined - > increase crossingPed, update timings
            if newPed:
                self.state += 1
                self.passed_pedestrians -= 1
                self.crossing_time -= self.state_time
                self.next_ped_time = 0.0
            # pedCrossed - > reduce crossingPed, update timings
            if pedCrossed:
                self.state -= 1
                self.crossing_time = 0.0
                self.next_ped_time -= self.state_time

        
    def calculate_next_pedestrian_time(self):
        num_ped_per_10_min = np.random.poisson(self.avgCrossingsPerTenMinutes)
        # x ped per 10 - > 10 / x min between ped - > 10 / x * 60 sec between ped
        minutes_next_ped = 10 / num_ped_per_10_min
        self.next_ped_time = minutes_next_ped * 60
        return self.next_ped_time
    
    def calculate_pedestrian_crossing_time(self):
        crossing_speed = np.random.lognormal(self.avgCrossingSpeed)
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
        self.state_time -= elapsed
        self.crossing_time -= elapsed
        self.next_ped_time -= elapsed
        
        self.relay_jam_control = 1


    # the output is called in the previous state.
    # the next state all ped crossed - > copy the state of the road ahead
    # update state - > copy road ahead
    # else, ped crossing - > should jam
    def output_control(self):
        all_ped_cross = (self.state == 1 and self.crossing_time < self.next_ped_time)
        if all_ped_cross:
            return self.road_jam_control
        elif self.relay_jam_control:
            return self.road_jam_control
        else:
            return {'RL': trafficInterface.TO_JAM, 'LR': trafficInterface.TO_JAM}


class PedestrianCrossingModel(AtomicDEVS):
    '''
    Constructor for a pedestrian crossing

    Attributes:
    :param num_pedestrians: total of pedestrians crossings to occur
    :param avgCrossingsPerTenMinutes: average crossings per 10 minutes
    :param avgCrossingSpeed: average crossing speed in m/s
    :return: none 
    '''
    def __init__(self, avgCrossingsPerTenMinutes, avgCrossingSpeed, num_pedestrians = INFINITY):
        AtomicDEVS.__init__(self, "PedestrianCrossing")
        self.IN_NEXT_JAM_LR = self.addInPort("in_jam_control_lr")
        self.OUT_JAM_LR = self.addOutPort("out_jam_control_lr")
        self.IN_NEXT_JAM_RL= self.addInPort("in_jam_control_rl")
        self.OUT_JAM_RL = self.addOutPort("out_jam_control_rl")
        self.state = PedestrianCrossingState(num_pedestrians, avgCrossingsPerTenMinutes = avgCrossingsPerTenMinutes, avgCrossingSpeed = avgCrossingSpeed)

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
    
    
