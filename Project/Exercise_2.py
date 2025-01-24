from pypdevs.DEVS import CoupledDEVS

from carSink import CarSink
from carGenerator import CarGeneratorModel
from pedestrianCrossing import PedestrianCrossingModel
from lightCrossing import LightCrossingModel
from roadsection import RoadSectionState, RoadSectionModel
from trafficInterface import JAMMED, FLUID

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

        self.actualMode()
        # self.testPedestrianCrossing()
        # self.testOneRoad()
        # self.testLightCrossing()

    def testPedestrianCrossing(self):
            sectionRL1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionRL1", max_speed=70, length=5, initial_state=FLUID), "sectionRL1"))
            sectionRL2 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionRL2", max_speed=70, length=10, initial_state=FLUID), "sectionRL2"))
            # sectionLR1 = self.addSubModel(
            # RoadSectionModel(RoadSectionState(name="sectionLR1", max_speed=70, length=10, initial_state=JAMMED), "sectionLR1"))

            carGeneratorRL = self.addSubModel(CarGeneratorModel(time_next_car=5, identifier=1))
            # carGeneratorLR = self.addSubModel(CarGeneratorModel(idle_time=10, identifier=2))

            pedestrianCrossing = self.addSubModel(PedestrianCrossingModel(avgCrossingsPerTenMinutes=200, avgCrossingSpeed = 1.4))

            self.connectPorts(carGeneratorRL.car_out, sectionRL1.IN_CAR)
            self.connectPorts(sectionRL1.OUT_CAR, sectionRL2.IN_CAR)
            # self.connectPorts(carGeneratorLR.car_out, sectionLR1.IN_CAR)

            self.connectPorts(sectionRL1.OUT_JAM, carGeneratorRL.IN_NEXT_JAM)
            # self.connectPorts(sectionLR1.OUT_JAM, carGeneratorLR.IN_NEXT_JAM)

            self.connectPorts(pedestrianCrossing.OUT_JAM_RL, sectionRL1.IN_NEXT_JAM)
            # self.connectPorts(pedestrianCrossing.OUT_JAM_LR, carGeneratorLR.IN_NEXT_JAM)

            self.connectPorts(sectionRL2.OUT_JAM, pedestrianCrossing.IN_NEXT_JAM_RL)
            # self.connectPorts(sectionLR1.OUT_JAM, pedestrianCrossing.IN_NEXT_JAM_LR)
    
    def testLightCrossing(self):
        sectionRL1 = self.addSubModel(
        RoadSectionModel(RoadSectionState(name="sectionRL1", max_speed=70, length=5, initial_state=FLUID), "sectionRL1"))
        sectionRL2 = self.addSubModel(
        RoadSectionModel(RoadSectionState(name="sectionRL2", max_speed=70, length=10, initial_state=FLUID), "sectionRL2"))
        # sectionLR1 = self.addSubModel(
        # RoadSectionModel(RoadSectionState(name="sectionLR1", max_speed=70, length=10, initial_state=JAMMED), "sectionLR1"))

        carGeneratorRL = self.addSubModel(CarGeneratorModel(time_next_car=5, identifier=1))
        # carGeneratorLR = self.addSubModel(CarGeneratorModel(idle_time=10, identifier=2))

        lightCrossing = self.addSubModel(LightCrossingModel(t_green=140, t_yellow=30, t_red=100))

        self.connectPorts(carGeneratorRL.car_out, sectionRL1.IN_CAR)
        self.connectPorts(sectionRL1.OUT_CAR, sectionRL2.IN_CAR)
        # self.connectPorts(carGeneratorLR.car_out, sectionLR1.IN_CAR)

        self.connectPorts(sectionRL1.OUT_JAM, carGeneratorRL.IN_NEXT_JAM)
        # self.connectPorts(sectionLR1.OUT_JAM, carGeneratorLR.IN_NEXT_JAM)

        self.connectPorts(lightCrossing.OUT_JAM_RL, sectionRL1.IN_NEXT_JAM)
        # self.connectPorts(pedestrianCrossing.OUT_JAM_LR, carGeneratorLR.IN_NEXT_JAM)

        self.connectPorts(sectionRL2.OUT_JAM, lightCrossing.IN_NEXT_JAM_RL)
        # self.connectPorts(sectionLR1.OUT_JAM, pedestrianCrossing.IN_NEXT_JAM_LR)

    def testOneRoad(self):
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
        
        self.carGeneratorR = self.addSubModel(CarGeneratorModel(time_next_car=10, identifier=1))
        self.carSinkR = self.addSubModel(CarSink(output_file='outfile.csv'))

        self.pedestrianCrossing = self.addSubModel(PedestrianCrossingModel(avgCrossingsPerTenMinutes=120, avgCrossingSpeed = 1.4))

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
        self.connectPorts(self.sectionR3_2.OUT_JAM, self.pedestrianCrossing.IN_NEXT_JAM_RL)
        self.connectPorts(self.pedestrianCrossing.OUT_JAM_RL, self.sectionR3_1.IN_NEXT_JAM)
        self.connectPorts(self.sectionR3_1.OUT_JAM, self.sectionR2.IN_NEXT_JAM)
        self.connectPorts(self.sectionR2.OUT_JAM, self.sectionR1.IN_NEXT_JAM)
        # # Couple exit to car sink
        self.connectPorts(self.sectionR5.OUT_CAR, self.carSinkR.in_car)




    def actualMode(self):
        # Declare the coupled model's output ports:
        # Autonomous, so no output ports

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

        self.carGeneratorR = self.addSubModel(CarGeneratorModel(time_next_car=10, identifier=1))
        self.carSinkR = self.addSubModel(CarSink(output_file='outfile.csv'))
        self.carGeneratorL = self.addSubModel(CarGeneratorModel(time_next_car=10, identifier=2))
        self.carSinkL = self.addSubModel(CarSink(output_file='outfile.csv'))

        #self.pedestrianCrossing = self.addSubModel(PedestrianCrossingModel(avgCrossingsPerTenMinutes=120, avgCrossingSpeed = 1.4))
        self.pedestrianCrossing = self.addSubModel(LightCrossingModel(t_green=140, t_yellow=30, t_red=100))

        
        

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
        self.connectPorts(self.sectionR3_2.OUT_JAM, self.pedestrianCrossing.IN_NEXT_JAM_RL)
        self.connectPorts(self.pedestrianCrossing.OUT_JAM_RL, self.sectionR3_1.IN_NEXT_JAM)
        self.connectPorts(self.sectionR3_1.OUT_JAM, self.sectionR2.IN_NEXT_JAM)
        self.connectPorts(self.sectionR2.OUT_JAM, self.sectionR1.IN_NEXT_JAM)
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
        self.connectPorts(self.sectionL3_2.OUT_JAM, self.pedestrianCrossing.IN_NEXT_JAM_LR)
        self.connectPorts(self.pedestrianCrossing.OUT_JAM_LR, self.sectionL3_1.IN_NEXT_JAM)
        self.connectPorts(self.sectionL3_1.OUT_JAM, self.sectionL2.IN_NEXT_JAM)
        self.connectPorts(self.sectionL2.OUT_JAM, self.sectionL1.IN_NEXT_JAM)
        # Couple exit to car sin
        self.connectPorts(self.sectionL5.OUT_CAR, self.carSinkL.in_car)
    


