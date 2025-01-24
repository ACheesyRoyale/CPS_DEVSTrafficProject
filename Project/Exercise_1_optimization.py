from pypdevs.DEVS import CoupledDEVS


from carSink import CarSink
from carGenerator import CarGeneratorModel
from roadsection import RoadSectionState, RoadSectionModel
from trafficInterface import JAMMED_OUTPUT, FULL_JAM, JAMMED, FLUID, FULL_JAM_OUTPUT, JAMMED_TO_FULL_JAM

class SimpleRoadModel(CoupledDEVS):
    def __init__(self):
        """
        A simple road consisting of 5 road sections :
            Section 1 & 5 : 70kph, 2500m,
            Section 2 & 4 : 50kph, 800m,
            Section 3 : 30 kph, 200m.
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, "system")

        # Declare the coupled model's output ports:
        # Autonomous, so no output ports

        # # Declare the coupled model's sub-models and add models to the coupled model:
        # Road left to right
        self.sectionR1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR1", max_speed=70, length=2500, initial_state=FLUID), "sectionR1"))
        self.sectionR2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR2", max_speed=50, length=800, initial_state=FLUID), "sectionR2"))
        self.sectionR3 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR3", max_speed=30, length=200, initial_state=FLUID), "sectionR3"))
        self.sectionR4 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR4", max_speed=50, length=800, initial_state=FLUID), "sectionR4"))
        self.sectionR5 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionR5", max_speed=70, length=2500, initial_state=FLUID), "sectionR5"))
        
        self.carGeneratorR = self.addSubModel(CarGeneratorModel(time_next_car=500, identifier=1, useFixedTime = False))
        self.carSinkR = self.addSubModel(CarSink(output_file='simpleH0.csv'))
        
        self.sectionL1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL1", max_speed=70, length=2500, initial_state=FLUID), "sectionL1"))
        self.sectionL2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL2", max_speed=50, length=800, initial_state=FLUID), "sectionL2"))
        self.sectionL3 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL3", max_speed=30, length=200, initial_state=FLUID), "sectionL3"))
        self.sectionL4 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL4", max_speed=50, length=800, initial_state=FLUID), "sectionL4"))
        self.sectionL5 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL5", max_speed=70, length=2500, initial_state=FLUID), "sectionL5"))
        
        self.carGeneratorL = self.addSubModel(CarGeneratorModel(time_next_car=500, identifier=2, useFixedTime = False))
        self.carSinkL = self.addSubModel(CarSink(output_file='simpleH0.csv'))

        
        

        # Couple the car generator's output port to the road section's input port
        self.connectPorts(self.carGeneratorR.car_out, self.sectionR1.IN_CAR)
        # connect the roads
        self.connectPorts(self.sectionR1.OUT_CAR, self.sectionR2.IN_CAR)
        self.connectPorts(self.sectionR2.OUT_CAR, self.sectionR3.IN_CAR)
        self.connectPorts(self.sectionR3.OUT_CAR, self.sectionR4.IN_CAR)
        self.connectPorts(self.sectionR4.OUT_CAR, self.sectionR5.IN_CAR)

        self.connectPorts(self.sectionR5.OUT_JAM, self.sectionR4.IN_NEXT_JAM)
        self.connectPorts(self.sectionR4.OUT_JAM, self.sectionR3.IN_NEXT_JAM)
        self.connectPorts(self.sectionR3.OUT_JAM, self.sectionR2.IN_NEXT_JAM)
        self.connectPorts(self.sectionR2.OUT_JAM, self.sectionR1.IN_NEXT_JAM)
        # Couple exit to car sink
        self.connectPorts(self.sectionR5.OUT_CAR, self.carSinkR.in_car)

        # Couple the car generator's output port to the road section's input port
        self.connectPorts(self.carGeneratorL.car_out, self.sectionL1.IN_CAR)
        # connect the roads
        self.connectPorts(self.sectionL1.OUT_CAR, self.sectionL2.IN_CAR)
        self.connectPorts(self.sectionL2.OUT_CAR, self.sectionL3.IN_CAR)
        self.connectPorts(self.sectionL3.OUT_CAR, self.sectionL4.IN_CAR)
        self.connectPorts(self.sectionL4.OUT_CAR, self.sectionL5.IN_CAR)

        self.connectPorts(self.sectionL5.OUT_JAM, self.sectionL4.IN_NEXT_JAM)
        self.connectPorts(self.sectionL4.OUT_JAM, self.sectionL3.IN_NEXT_JAM)
        self.connectPorts(self.sectionL3.OUT_JAM, self.sectionL2.IN_NEXT_JAM)
        self.connectPorts(self.sectionL2.OUT_JAM, self.sectionL1.IN_NEXT_JAM)
        # Couple exit to car sin
        self.connectPorts(self.sectionL5.OUT_CAR, self.carSinkL.in_car)
    


