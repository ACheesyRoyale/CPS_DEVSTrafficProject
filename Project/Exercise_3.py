from pypdevs.DEVS import CoupledDEVS


from carSink import CarSink
from carGenerator import CarGeneratorModel
from intersectionRoad import IntersectionRoad, IntersectionRoadState
from roadsection import RoadSectionModel, RoadSectionState
from trafficInterface import FLUID

class SimpleRoadModel(CoupledDEVS):
    def __init__(self):
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, "system")

        #self.testIntersectionRoad()
        # self.testFilter()
        self.testMainTwoParallelBiDirectionNoLights()

    def testIntersectionRoad(self):
        carGeneratorR = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=1, pLocal=0.7))
        carSinkT = self.addSubModel(CarSink(output_file='outfile.csv'))
        carGeneratorL = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=2, pLocal=0.5))
        intersectionRoad = self.addSubModel(IntersectionRoad(state=IntersectionRoadState(), name="test"))
        sectionL1 = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="sectionL1", max_speed=70, length=250, initial_state=FLUID), "sectionL1"))
        
        # Couple the car generator's output port to the road section's input port
        self.connectPorts(carGeneratorR.car_out, intersectionRoad.IN_CAR_RIGHT)
        self.connectPorts(carGeneratorL.car_out, intersectionRoad.IN_CAR_LEFT)
        
        # Couple exit to car sin
        self.connectPorts(intersectionRoad.OUT_CAR_TOP, sectionL1.IN_CAR)
        self.connectPorts(sectionL1.OUT_CAR, carSinkT.in_car)

    def testFilter(self):
        # generatorR1 = 2, generatorL1 = 1

        ## map : Intersection : {destination : [Local, NotLocal]}
        intersectionFilterMap = {"LM" : {2: ["R", "TB"]}}
        
        carGeneratorDestinations = {1 : [2]}

        carGeneratorL = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=1, pLocal=0.7, carDestinationMap = carGeneratorDestinations[1]))
        carSinkT = self.addSubModel(CarSink(output_file='outfileT.csv'))
        carSinkB = self.addSubModel(CarSink(output_file='outfileB.csv'))
        carSinkR = self.addSubModel(CarSink(output_file='outfileR.csv'))
        intersectionRoad = self.addSubModel(IntersectionRoad(name="LM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["LM"])))

        
        # Couple the car generator's output port to the road section's input port
        self.connectPorts(carGeneratorL.car_out, intersectionRoad.IN_CAR_RIGHT)
        
        # Couple exit to car sin
        self.connectPorts(intersectionRoad.OUT_CAR_TOP, carSinkT.in_car)
        self.connectPorts(intersectionRoad.OUT_CAR_BOT, carSinkB.in_car)
        self.connectPorts(intersectionRoad.OUT_CAR_RIGHT, carSinkR.in_car)

    def testMainTwoParallelBiDirectionNoLights(self):
       # generatorR1 = 2, generatorL1 = 1

        ## map : Intersection : {destination : [Local, NotLocal]}
        intersectionFilterMap = {"LM" : {2: ["R", "TB"], 1: ["L", "L"]}, 
                                 "RM": {2: ["R", "R"], 1: ["L", "TB"]}}
        
        carGeneratorDestinations = {1 : [2], 2 : [1]}

        carGeneratorLM = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=1, pLocal=0.7, carDestinationMap = carGeneratorDestinations[1]))
        carSinkRM = self.addSubModel(CarSink(output_file='outfileR.csv'))
        carGeneratorRM = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=2, pLocal=0.7, carDestinationMap = carGeneratorDestinations[2]))
        carSinkLM = self.addSubModel(CarSink(output_file='outfileL.csv'))
        intersectionRoadLM = self.addSubModel(IntersectionRoad(name="LM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["LM"])))
        intersectionRoadRM = self.addSubModel(IntersectionRoad(name="RM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["RM"])))
        sectionMain_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Main_LR", max_speed=30, length=250, initial_state=FLUID), "Main_LR"))
        sectionMain_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Main_RL", max_speed=30, length=250, initial_state=FLUID), "Main_RL"))
        
        sectionParallel_T1_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_T1_LR", max_speed=70, length=450, initial_state=FLUID), "Parallel_T1_LR"))
        sectionParallel_T1_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_T1_RL", max_speed=70, length=450, initial_state=FLUID), "Parallel_T1_RL"))
        
        sectionParallel_B1_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_B1_LR", max_speed=70, length=450, initial_state=FLUID), "Parallel_B1_LR"))
        sectionParallel_B1_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_B1_RL", max_speed=70, length=450, initial_state=FLUID), "Parallel_B1_RL"))
        
        
        # Left Generator/Sink combination
        self.connectPorts(carGeneratorLM.car_out, intersectionRoadLM.IN_CAR_LEFT)
        self.connectPorts(intersectionRoadLM.OUT_CAR_LEFT, carSinkLM.in_car)
        
        # Intersection LeftMid
        self.connectPorts(intersectionRoadLM.OUT_CAR_TOP, sectionParallel_T1_LR.IN_CAR)
        self.connectPorts(intersectionRoadLM.OUT_CAR_BOT, sectionParallel_B1_LR.IN_CAR)
        self.connectPorts(intersectionRoadLM.OUT_CAR_RIGHT, sectionMain_LR.IN_CAR) 

        self.connectPorts(sectionParallel_T1_RL.OUT_CAR, intersectionRoadLM.IN_CAR_TOP)
        self.connectPorts(sectionParallel_B1_RL.OUT_CAR, intersectionRoadLM.IN_CAR_BOT)
        self.connectPorts(sectionMain_RL.OUT_CAR, intersectionRoadLM.IN_CAR_RIGHT) 

        # Intersection RightMid
        self.connectPorts(intersectionRoadRM.OUT_CAR_TOP, sectionParallel_T1_RL.IN_CAR)
        self.connectPorts(intersectionRoadRM.OUT_CAR_BOT, sectionParallel_B1_RL.IN_CAR)
        self.connectPorts(intersectionRoadRM.OUT_CAR_LEFT, sectionMain_RL.IN_CAR) 

        self.connectPorts(sectionParallel_T1_LR.OUT_CAR, intersectionRoadRM.IN_CAR_TOP)
        self.connectPorts(sectionParallel_B1_LR.OUT_CAR, intersectionRoadRM.IN_CAR_BOT)
        self.connectPorts(sectionMain_LR.OUT_CAR, intersectionRoadRM.IN_CAR_LEFT) 

        # Left Generator/Sink Combination
        self.connectPorts(carGeneratorRM.car_out, intersectionRoadRM.IN_CAR_RIGHT)
        self.connectPorts(intersectionRoadRM.OUT_CAR_RIGHT, carSinkRM.in_car)
        