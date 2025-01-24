from pypdevs.DEVS import CoupledDEVS

from carSink import CarSink
from carGenerator import CarGeneratorModel
from pedestrianCrossing import PedestrianCrossingModel
from lightCrossing import LightCrossingModel
from roadsection import RoadSectionState, RoadSectionModel
from trafficInterface import JAMMED, FLUID

class SimpleRoadModel(CoupledDEVS):
    def __init__(self, time_next_car, output_file, light_timings, num_traffic_lights):
        CoupledDEVS.__init__(self, "system")

        self.time_next_car = time_next_car
        self.output_file = output_file
        self.light_timings = light_timings  # A list of tuples (green, yellow, red) for each light
        self.num_traffic_lights = num_traffic_lights

        # Create traffic lights
        self.traffic_lights = []
        for i in range(self.num_traffic_lights):
            t_green, t_yellow, t_red = self.light_timings[i]
            traffic_light = self.addSubModel(LightCrossingModel(t_green=t_green, t_yellow=t_yellow, t_red=t_red))
            self.traffic_lights.append(traffic_light)

        self.actualMode()


    def actualMode(self):
        # # Declare the coupled model's sub-models and add models to the coupled model:
        # Road left to right
        self.sectionR1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR1", max_speed=70, length=2500, initial_state=FLUID), "sectionR1"))
        self.sectionR2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR2", max_speed=50, length=800, initial_state=FLUID), "sectionR2"))
        self.sectionR3_1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR3_1", max_speed=30, length=100, initial_state=FLUID), "sectionR3_1"))
        self.sectionR3_2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR3_2", max_speed=30, length=100, initial_state=FLUID), "sectionR3_2"))
        self.sectionR4 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR4", max_speed=50, length=800, initial_state=FLUID), "sectionR4"))
        self.sectionR5 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR5", max_speed=70, length=2500, initial_state=FLUID), "sectionR5"))
        # Road right to left
        self.sectionL1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL1", max_speed=70, length=2500, initial_state=FLUID), "sectionL1"))
        self.sectionL2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL2", max_speed=50, length=800, initial_state=FLUID), "sectionL2"))
        self.sectionL3_1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL3_1", max_speed=30, length=100, initial_state=FLUID), "sectionL3_1"))
        self.sectionL3_2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL3_2", max_speed=30, length=100, initial_state=FLUID), "sectionL3_2"))
        self.sectionL4 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL4", max_speed=50, length=800, initial_state=FLUID), "sectionL4"))
        self.sectionL5 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL5", max_speed=70, length=2500, initial_state=FLUID), "sectionL5"))

        # Define generators and sinks
        self.carGeneratorR = self.addSubModel(CarGeneratorModel(time_next_car=self.time_next_car, identifier=1, useFixedTime=False))
        self.carSinkR = self.addSubModel(CarSink(output_file=self.output_file))
        self.carGeneratorL = self.addSubModel(CarGeneratorModel(time_next_car=self.time_next_car, identifier=2, useFixedTime=False))
        self.carSinkL = self.addSubModel(CarSink(output_file=self.output_file))

#modify all the stuff to actually connect the lights correctly

        # Crossing zebra or light controlled
        #self.pedestrianCrossing = self.addSubModel(PedestrianCrossingModel(avgCrossingsPerTenMinutes=150, avgCrossingSpeed = 1.4))
        #self.pedestrianCrossing = self.addSubModel(LightCrossingModel(t_green=self.t_green, t_yellow=self.t_yellow, t_red=self.t_red))
        # Create the traffic light models for each intersection

        #coupling examples
        #self.connectPorts(self.sectionR3_2.OUT_CAR, self.traffic_lights[0].IN_CAR)  # Connecting section R3 to the first traffic light
        #self.connectPorts(self.traffic_lights[0].OUT_CAR, self.sectionR4.IN_CAR)    # Connecting traffic light output to section R4

        # Couple the car generator's output port to the road section's input port
        self.connectPorts(self.carGeneratorR.car_out, self.sectionR1.IN_CAR)
        # connect the roads
        self.connectPorts(self.sectionR1.OUT_CAR, self.sectionR2.IN_CAR)
        self.connectPorts(self.sectionR2.OUT_CAR, self.sectionR3_1.IN_CAR)
        self.connectPorts(self.sectionR3_1.OUT_CAR, self.sectionR3_2.IN_CAR)
        self.connectPorts(self.sectionR3_2.OUT_CAR, self.sectionR4.IN_CAR)
        self.connectPorts(self.sectionR4.OUT_CAR, self.sectionR5.IN_CAR)

        self.connectPorts(self.sectionR5.OUT_JAM, self.sectionR4.IN_NEXT_JAM)
        self.connectPorts(self.sectionR4.OUT_JAM, self.sectionR3_2.IN_NEXT_JAM)
        self.connectPorts(self.sectionR3_2.OUT_JAM, self.traffic_lights[0].IN_NEXT_JAM_RL)
        self.connectPorts(self.traffic_lights[0].OUT_JAM_RL, self.sectionR3_1.IN_NEXT_JAM)
        self.connectPorts(self.sectionR3_1.OUT_JAM, self.sectionR2.IN_NEXT_JAM)
        self.connectPorts(self.sectionR2.OUT_JAM, self.traffic_lights[1].IN_NEXT_JAM_RL)
        self.connectPorts(self.traffic_lights[1].OUT_JAM_RL, self.sectionR1.IN_NEXT_JAM)
        # # Couple exit to car sink
        self.connectPorts(self.sectionR5.OUT_CAR, self.carSinkR.in_car)

        # Couple the car generator's output port to the road section's input port
        self.connectPorts(self.carGeneratorL.car_out, self.sectionL1.IN_CAR)
        # connect the roads
        self.connectPorts(self.sectionL1.OUT_CAR, self.sectionL2.IN_CAR)
        self.connectPorts(self.sectionL2.OUT_CAR, self.sectionL3_1.IN_CAR)
        self.connectPorts(self.sectionL3_1.OUT_CAR, self.sectionL3_2.IN_CAR)
        self.connectPorts(self.sectionL3_2.OUT_CAR, self.sectionL4.IN_CAR)
        self.connectPorts(self.sectionL4.OUT_CAR, self.sectionL5.IN_CAR)

        self.connectPorts(self.sectionL5.OUT_JAM, self.sectionL4.IN_NEXT_JAM)
        self.connectPorts(self.sectionL4.OUT_JAM, self.sectionL3_2.IN_NEXT_JAM)
        self.connectPorts(self.sectionL3_2.OUT_JAM, self.traffic_lights[0].IN_NEXT_JAM_LR)
        self.connectPorts(self.traffic_lights[0].OUT_JAM_LR, self.sectionL3_1.IN_NEXT_JAM)
        self.connectPorts(self.sectionL3_1.OUT_JAM, self.sectionL2.IN_NEXT_JAM)
        self.connectPorts(self.sectionL2.OUT_JAM, self.traffic_lights[1].IN_NEXT_JAM_LR)
        self.connectPorts(self.traffic_lights[1].OUT_JAM_LR, self.sectionL1.IN_NEXT_JAM)
        # Couple exit to car sin
        self.connectPorts(self.sectionL5.OUT_CAR, self.carSinkL.in_car)
    


