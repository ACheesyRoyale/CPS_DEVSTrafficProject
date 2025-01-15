from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

from car import Car

import trafficInterface
from trafficInterface import JAMMED, IDLE, GENERATING

import numpy as np

class CarGeneratorState1:
    def __init__(self, identifier, num_cars, time_next_car, useFixedTime):
        self.state = IDLE
        self.time_next_car = time_next_car
        self.car_identifier = identifier * 1000 + -1
        self.car_creation_timer = 0.0
        self.number_of_cars = num_cars
        self.useFixedTime = useFixedTime
        self.idleTime = 0.0

    def __str__(self):
        return "*** state = " + str(self.state) + " num of cars left " + str(self.number_of_cars)

    def internal(self):
        if self.state == IDLE:
            self.state = GENERATING
        elif self.state == GENERATING:
            self.state = IDLE

    def calculate_time_advance(self):
        if self.number_of_cars <= 0:
            return INFINITY
        
        if self.useFixedTime:
            self.idleTime = {IDLE: self.time_next_car,
                    GENERATING: 0,
                    JAMMED: INFINITY
                    }[self.state]
        else:
            self.idleTime = {IDLE: self.calculate_next_car_time(),
                    GENERATING: 0,
                    JAMMED: INFINITY
                    }[self.state]
        return self.idleTime

    def output_car(self):
        print("carGen outputFnc")
        if self.state == GENERATING:
            self.car_creation_timer += self.idleTime
            self.number_of_cars -= 1
            self.car_identifier += 1
            return Car(speed_adapter=1, creation_time=self.car_creation_timer, id=self.car_identifier)
        
        return None
    
    def solve_jam_control(self, jam_control):
        if jam_control is trafficInterface.TO_JAM:
            self.state = JAMMED
        if jam_control is trafficInterface.TO_FLUID:
            self.state = IDLE

    def calculate_next_car_time(self):
        return np.random.poisson(self.idleTime)

class CarGeneratorState:
    def __init__(self, identifier, num_cars, time_next_car, useFixedTime, percentLocal = 0):
        self.state = num_cars
        self.car_identifier = identifier * 1000 + -1
        self.time_next_car = time_next_car
        self.state_time = 0.0
        self.next_time = 0.0
        self.useFixedTime = useFixedTime
        self.car_creation_timer = 0.0
        self.isJammed = False
        self.percentLocal = percentLocal

    def internal(self):
        self.state -= 1
        return self.state
    
    def calculate_time_advance(self):
        # all cars passed
        if self.state <= 0:
            return INFINITY
        
        # generator jammed by traffic
        if self.isJammed:
            return INFINITY
        
        # extTrans interrupted but interruption shorter than generation : continue
        if self.next_time > 0:
            return self.next_time

        # carGeneration is either on a fixed interval or according to the randomization function.
        if self.useFixedTime:
            self.next_time = self.time_next_car
        else:
            self.next_time = self.calculate_next_car_time()

        self.state_time = self.next_time
        return self.state_time

    def calculate_next_car_time(self):
        return np.random.poisson(self.time_next_car)

    def output_car(self):
        print("carGen outputFnc")
        if not self.isJammed:
            self.car_creation_timer += self.state_time
            self.next_time = 0
            self.car_identifier += 1
            isLocal = bool(np.random.choice([0,1], p=[1-self.percentLocal, self.percentLocal]))
            return Car(speed_adapter=1, creation_time=self.car_creation_timer, id=self.car_identifier, isLocal=isLocal)
    
    def solve_jam_control(self, jam_control, elapsed):
        # traffic jam blocks generator.
        if jam_control is trafficInterface.TO_JAM:
            self.isJammed = True

        # traffic jam cleared
        if jam_control is trafficInterface.TO_FLUID:
            self.isJammed = False
        
        # remove elapsed time from next_time
        self.next_time -= elapsed


class CarGeneratorModel(AtomicDEVS):
    def __init__(self, time_next_car, identifier, num_cars = INFINITY, useFixedTime = True, pLocal = 0):
        AtomicDEVS.__init__(self, "CarGenerator"+str(identifier))
        self.car_out = self.addOutPort("car_out")
        self.IN_NEXT_JAM = self.addInPort("next_section_jam")

        self.state = CarGeneratorState(identifier, num_cars, time_next_car, useFixedTime, percentLocal=pLocal)
        if not isinstance(self.state, CarGeneratorState):
            print("error init.")
            exit(1)

    # defines the next state for each state
    # This is the only function allowed to change the state.
    def intTransition(self):
        self.state.internal()
        return self.state

    def __str__(self):
        return super().__str__()
    
    # defines the delay in each state
    # self.state is read-only
    def timeAdvance(self):
        return self.state.calculate_time_advance()

    # Receives JAM_Control from next section
    def extTransition(self, inputs):
        jam_control = None
        if self.IN_NEXT_JAM in inputs:
            jam_control = inputs[self.IN_NEXT_JAM]
            print("carGen extTransition | Jam control: ", jam_control) # debug print
        if jam_control is not None:
            self.state.solve_jam_control(jam_control, self.elapsed)
        return self.state

    # is called right before the internal change
    def outputFnc(self):
        rv = {}
        car = self.state.output_car()
        if car is not None:
            rv [self.car_out] = car

        return rv
