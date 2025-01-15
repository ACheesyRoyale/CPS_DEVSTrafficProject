from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY

import trafficInterface
from trafficInterface import JAMMED_OUTPUT, FULL_JAM, JAMMED, FLUID, FULL_JAM_OUTPUT, JAMMED_TO_FULL_JAM


class RoadSectionState:
    """Road Section Model State class:

    Defines a basic single lane state
    """
    def __init__(self, length, max_speed, initial_state, name):
        """
        Constructor for a road section
        :param length: The length of the road-section in meters
        :param max_speed: The maximum speed allowed on the road_section
        :return: none
        """
        self.length = length
        self.max_speed = max_speed
        self.state = initial_state
        self.queue = []
        self.jam_queue = []
        self.next_transition = INFINITY
        self.name = name

    def __str__(self):
        theRv = "*** " + self.name + " state = " + str(self.state) + " ***\nlength = " + str(self.length) + " meter\nJam: " + str(self.calculate_jam_length()) + " meter"
        theRv += "\nnr_driving = " +str(len(self.queue)) + " nr_jammed =  " + str(len(self.jam_queue))
        return theRv

    def calculate_jam_length(self):
        """
        Average length of car is 4.5 meter, additional 0.5 meter for space in between cars.
        """
        return len(self.jam_queue) * 5

    def internal(self):
        """
        internal transitions only happen when a car leaves the roadsection or
        when the car joins the traffic jam
        * When FLUID: The output function is called just before and thus already transfered a car to the next section
        it just needs to be removed here now
        * When JAMMED: The output function does not transfer car to the next section but to the jamqueue
        * When JAMMED_output: The first car from the jamqueue is outputed. It just needs to be removed here
        * When FULL_JAM: impossible?
        * When FULL_JAM_OUTPUT =  The first car from the jamqueue is transfered to the next roadsection or intersection
          the outputfunction should have taken car of this. Here is is just removed
        * When JAMMED_TO_FULL_JAM = just
        """
        # Go through all cars in the queue and move them:
        # print("roadSection internal | (name, state):", self.name, ",", self.state)
        for car in self.queue:
            car.current_position_on_segment = car.current_position_on_segment + trafficInterface.calculate_distance_from_time_speed(
                self.max_speed, self.next_transition)
        if self.state is FLUID:
            self.queue = self.queue[1:]
        elif self.state is JAMMED:
            first_car = self.queue[0]
            self.queue = self.queue[1:]
            self.jam_queue.append(first_car)
            if self.calculate_jam_length() >= self.length:
                self.state = JAMMED_TO_FULL_JAM
        elif self.state is FULL_JAM:
            print("internal transition on FULL_JAM: This should not happen")
        elif self.state is JAMMED_OUTPUT:
            self.jam_queue = self.jam_queue[1:]
        elif self.state is FULL_JAM_OUTPUT:
            self.jam_queue = self.jam_queue[1:]
        elif self.state is JAMMED_TO_FULL_JAM:
            self.state = FULL_JAM
        else:
            pass

    def calculate_time_advance(self):
        """
        Calculates the tima advance :
        If FLUID: calculate fir the first in queue, when it will arrive at the end
        if JAMMED: calculate when the first driving will arrive at the JAM
        if JAMMED_OUTPUT or FULL_JAM_OUTPUT =  ghost state to remove a car from the queue
        if FULL_JAM: infite
        """
        if self.state is FLUID or self.state is JAMMED:
            if len(self.queue) > 0:
                distance_to_cover = self.length - self.calculate_jam_length() - self.queue[0].current_position_on_segment
                self.next_transition = trafficInterface.calculate_time_from_distance_speed(self.max_speed, distance_to_cover)
            else:
                self.next_transition = INFINITY
        elif self.state is JAMMED_OUTPUT or self.state is FULL_JAM_OUTPUT or self.state is JAMMED_TO_FULL_JAM: # !!! Add JAMMED_TO_FULL_JAM
            self.next_transition = 0
        elif self.state is FULL_JAM:
            self.next_transition = INFINITY
        return self.next_transition

    def external_to_jam(self, elapsed):
        self.update_car_positions(elapsed)
        self.state = JAMMED

    def external_jam_solved(self, elapsed):
        # We will merge the two queues. At this point we make a big assumption (infinite acceleration)
        # first calculate the position in the queue and update it in the queue
        self.update_car_positions(elapsed)
        for idx,car in enumerate(self.jam_queue):
            car.current_position_on_segment = self.length - (idx*5)
        # Now merge the queues:
        self.queue = self.jam_queue + self.queue
        self.jam_queue = []
        self.state = FLUID


    def external_spot_available(self, elapsed):
        self.update_car_positions(elapsed)
        if self.state is JAMMED:
            self.state = JAMMED_OUTPUT
        elif self.state is FULL_JAM:
            self.state = FULL_JAM_OUTPUT
        else:
            print("error in external_spot_available as can only happen in JAMMED and FULL_JAM")


    def output_car(self):
        if self.state is JAMMED_OUTPUT or self.state is FULL_JAM_OUTPUT:
            self.jam_queue[0].distance_done += self.length
            return self.jam_queue[0]
        if self.state is FLUID:
            self.queue[0].distance_done += self.length
            return self.queue[0]
        if self.state is JAMMED or JAMMED_TO_FULL_JAM:
            return None

    def output_control(self):
        """
        logic to send out control signals
        IF JAMMED or FLUID or JAMMED_OUTPUT : no control needed upstream
        IF GOING_TO_FULL_JAM : warn upstream that this section is fully jammed
        IF GOING_TO_FLUID: To-implement
        IF FULL_JAM_OUTPUT: new spot available in this queue
        """
        # print("roadSection Output control | (name, state): ", self.name, self.state) # debug print
        if self.state is JAMMED_OUTPUT or self.state is FLUID or self.state is JAMMED:
            return None
        if self.state is JAMMED_TO_FULL_JAM or self.state is FULL_JAM:
            return trafficInterface.TO_JAM
        if self.state is FULL_JAM_OUTPUT:
            return trafficInterface.JAM_CAR_SPOT_AVAILABLE

    def car_enters(self, elapsed, car):
        """
        Logic when car enters
        """
        car.current_velocity = self.max_speed
        car.current_position_on_segment = 0 # car has to start at the beginning
        self.update_car_positions(elapsed)
        if self.state is FULL_JAM:
            self.jam_queue.append(car)
        if self.state is JAMMED:
            jam_length = self.calculate_jam_length() # current jam length
            if jam_length >= self.length-5: # no place for new car 
                self.jam_queue = self.jam_queue + self.queue
                self.jam_queue.append(car)
                self.queue = []
                self.state = JAMMED_TO_FULL_JAM
            else: # place for new car
                # if len(self.queue) > 0: # other cars driving?
                #     # fit new car behind last car?
                #     if self.queue[-1].current_position_on_segment >= 5: 
                #         car.current_position_on_segment = self.queue[-1].current_position_on_segment - 5
                self.queue.append(car)
        else:
            # if len(self.queue) > 0:
            #     if self.queue[-1].current_position_on_segment <= 5:
            #         car.current_position_on_segment = self.queue[-1].current_position_on_segment - 5
            self.queue.append(car)

    def update_car_positions(self,elapsed):
        """
        Update all the car position on an external event
        """
        for car in self.queue:
            car.current_position_on_segment = car.current_position_on_segment + trafficInterface.calculate_distance_from_time_speed(self.max_speed, elapsed)

class RoadSectionModel(AtomicDEVS):
    """Class docstrings go here."""

    def __init__(self, state, name):
        AtomicDEVS.__init__(self, name)
        if isinstance(state, RoadSectionState):
            self.state = state
        else:
            print("error in init of roadSectionModel")
            exit(1)
        self.IN_CAR = self.addInPort("incoming_car")
        self.IN_NEXT_JAM = self.addInPort("next_section_jam")
        self.OUT_CAR = self.addOutPort("outgoing_car")
        self.OUT_JAM = self.addOutPort("out_going_jam")

    def intTransition(self):
        self.state.internal()
        return self.state

    def extTransition(self, inputs):
        car = None
        jam_control = None
        if self.IN_CAR in inputs:
            car = inputs[self.IN_CAR]
            # print("roadSection extTransition | car: ", car) # debug print
        if self.IN_NEXT_JAM in inputs:
            jam_control = inputs[self.IN_NEXT_JAM]
            # print("roadSection extTransition | Jam control: ", jam_control) # debug print
        if car is not None:
            self.state.car_enters(self.elapsed,car)
        if jam_control is not None:
            if jam_control is trafficInterface.JAM_CAR_SPOT_AVAILABLE:
                self.state.external_spot_available(self.elapsed)
            elif jam_control is trafficInterface.TO_JAM:
                self.state.external_to_jam(self.elapsed)
            elif jam_control is trafficInterface.TO_FLUID:
                self.state.external_jam_solved(self.elapsed)
        return self.state

    def timeAdvance(self):
        return self.state.calculate_time_advance()

    def outputFnc(self):
        control = self.state.output_control()
        car = self.state.output_car()
        rv= {}
        if car is not None:
            rv[self.OUT_CAR] = car
        if control is not None:
            rv[self.OUT_JAM] = control
        return rv
