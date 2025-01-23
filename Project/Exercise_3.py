from pypdevs.DEVS import CoupledDEVS


from carSink import CarSink
from carGenerator import CarGeneratorModel
from intersectionRoad import IntersectionRoad, IntersectionRoadState
from roadsection import RoadSectionModel, RoadSectionState
from pedestrianCrossing import PedestrianCrossingModel
from trafficInterface import FLUID

class SimpleRoadModel(CoupledDEVS):
    def __init__(self):
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, "system")

        #self.testIntersectionRoad()
        # self.testFilter()
        # self.testMainTwoParallelBiDirectionNoLights()
        self.testExtendedCityNoLights()

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

        # / Connect Gen/Sink to intersect
        # // Gen/Sink LM to IntersectionLM
        carGeneratorLM = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=1, pLocal=0.7, carDestinationMap = carGeneratorDestinations[1]))
        carSinkLM = self.addSubModel(CarSink(output_file='outfileL.csv'))
        intersectionRoadLM = self.addSubModel(IntersectionRoad(name="LM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["LM"])))
        
        self.connectPorts(carGeneratorLM.car_out, intersectionRoadLM.IN_CAR_LEFT)
        self.connectPorts(intersectionRoadLM.OUT_CAR_LEFT, carSinkLM.in_car)

        # // Gen/Sink RM to Intersection RM
        carSinkRM = self.addSubModel(CarSink(output_file='outfileR.csv'))
        carGeneratorRM = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=2, pLocal=0.7, carDestinationMap = carGeneratorDestinations[2]))
        intersectionRoadRM = self.addSubModel(IntersectionRoad(name="RM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["RM"])))
    
        self.connectPorts(carGeneratorRM.car_out, intersectionRoadRM.IN_CAR_RIGHT)
        self.connectPorts(intersectionRoadRM.OUT_CAR_RIGHT, carSinkRM.in_car)

        # / Connect Main Street to Intersections
        # // LR traffic
        sectionMain_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Main_LR", max_speed=30, length=250, initial_state=FLUID), "Main_LR"))
        
        self.connectPorts(intersectionRoadLM.OUT_CAR_RIGHT, sectionMain_LR.IN_CAR) 
        self.connectPorts(sectionMain_LR.OUT_CAR, intersectionRoadRM.IN_CAR_LEFT) 

        # // RL traffic
        sectionMain_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Main_RL", max_speed=30, length=250, initial_state=FLUID), "Main_RL"))
        
        self.connectPorts(intersectionRoadRM.OUT_CAR_LEFT, sectionMain_RL.IN_CAR) 
        self.connectPorts(sectionMain_RL.OUT_CAR, intersectionRoadLM.IN_CAR_RIGHT) 
        
        # / Connect top parallel to intersections
        # // LR traffic
        sectionParallel_T1_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_T1_LR", max_speed=70, length=450, initial_state=FLUID), "Parallel_T1_LR"))
        
        self.connectPorts(intersectionRoadLM.OUT_CAR_TOP, sectionParallel_T1_LR.IN_CAR)
        self.connectPorts(sectionParallel_T1_LR.OUT_CAR, intersectionRoadRM.IN_CAR_TOP)

        # // RL traffic
        sectionParallel_T1_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_T1_RL", max_speed=70, length=450, initial_state=FLUID), "Parallel_T1_RL"))
        
        self.connectPorts(intersectionRoadRM.OUT_CAR_TOP, sectionParallel_T1_RL.IN_CAR)
        self.connectPorts(sectionParallel_T1_RL.OUT_CAR, intersectionRoadLM.IN_CAR_TOP)

        # / Connect bot parallel to intersections
        # // LR traffic
        sectionParallel_B1_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_B1_LR", max_speed=70, length=450, initial_state=FLUID), "Parallel_B1_LR"))
        
        self.connectPorts(intersectionRoadLM.OUT_CAR_BOT, sectionParallel_B1_LR.IN_CAR)
        self.connectPorts(sectionParallel_B1_LR.OUT_CAR, intersectionRoadRM.IN_CAR_BOT)

        # // RL traffic
        sectionParallel_B1_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="Parallel_B1_RL", max_speed=70, length=450, initial_state=FLUID), "Parallel_B1_RL"))
        
        self.connectPorts(sectionParallel_B1_RL.OUT_CAR, intersectionRoadLM.IN_CAR_BOT)
        self.connectPorts(intersectionRoadRM.OUT_CAR_BOT, sectionParallel_B1_RL.IN_CAR) 

    def testExtendedCityNoLights(self):
       # generatorR1 = 2, generatorL1 = 1

        ## map : Intersection : {destination : [Local, NotLocal]}
        # intersectionFilterMap = {"LM" : {2: ["R", "TB"], 1: ["L", "L"]}, 
        #                          "RM": {2: ["R", "R"], 1: ["L", "TB"]},}
        # cars generated at fixed interval
        genUseFixedTime = True
        # average interval between generated cars
        genAverageTime = 100
        # percent (0 to 1) of local cars
        genPrecLocal = 0.5

        # length in meters of various sections
        lenCenter_L = 800
        lenCenter_R = 800
        # horizontal mid sections same length
        lenCenter_M = 200
        lenTop_H = lenCenter_M
        lenBot_H = lenCenter_M
        # vertical sections same length
        lenVert_LT = 50
        lenVert_LB = 50
        lenVert_RT = 50
        lenVert_RB = 50
        # outer loops of city create block same size as inner blocks
        lenBot_B = lenCenter_M + 50 + 50
        lenTop_T = lenCenter_M + 50 + 50

        # map = {Intersection that filters: {DestinationGenerator : [Directions to send cars]}}
        intersectionFilterMap = {"LM": {"RM": ["R", "BT"], "LM": ["L", "L"]},
                                 "RM": {"LM": ["L", "BT"], "RM": ["R", "R"]},
                                 "LB": {"RM": ["T", "RB"], "LM": ["T", "T"]},
                                 "RB": {"LM": ["T", "RB"], "RM": ["T", "T"]},
                                 "LT": {"RM": ["B", "RT"], "LM": ["B", "B"]},
                                 "RT": {"LM": ["B", "LT"], "RM": ["B", "B"]}}
        
        # map = {Generator that generates cars : [Destination Generators for Cars]}
        carGeneratorDestinations = {"LM" : ["RM"], 
                                    "RM" : ["LM"]}

        # / Horizontal Road Segments
        # // Center segment
        # /// Construct Main Street with Ped Crossing
        pedestrianCrossing = self.addSubModel(PedestrianCrossingModel(avgCrossingsPerTenMinutes=120, avgCrossingSpeed = 1.4))
        # IV. LR traffic 
        sectionCenter_MainLeft_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_ML_LR", max_speed=30, length=lenCenter_M/2, initial_state=FLUID), "RoadC_ML_LR"))
        sectionCenter_MainRight_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_MR_LR", max_speed=30, length=lenCenter_M/2, initial_state=FLUID), "RoadC_MR_LR"))
        
        # V. Car connect
        self.connectPorts(sectionCenter_MainLeft_LR.OUT_CAR, sectionCenter_MainRight_LR.IN_CAR)
        # V. Control connect
        self.connectPorts(sectionCenter_MainLeft_LR.OUT_JAM, pedestrianCrossing.IN_NEXT_JAM_LR)
        self.connectPorts(pedestrianCrossing.OUT_JAM_LR, sectionCenter_MainRight_LR.IN_NEXT_JAM)

        # IV. RL Traffic
        sectionCenter_MainRight_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_MR_RL", max_speed=30, length=lenCenter_M/2, initial_state=FLUID), "RoadC_MR_RL"))
        sectionCenter_MainLeft_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_ML_RL", max_speed=30, length=lenCenter_M/2, initial_state=FLUID), "RoadC_ML_RL"))
        
        # V. Car Connect
        self.connectPorts(sectionCenter_MainRight_RL.OUT_CAR, sectionCenter_MainLeft_RL.IN_CAR)
        # V. Control Connect
        self.connectPorts(sectionCenter_MainRight_RL.OUT_JAM, pedestrianCrossing.IN_NEXT_JAM_RL)
        self.connectPorts(pedestrianCrossing.OUT_JAM_RL, sectionCenter_MainLeft_RL.IN_NEXT_JAM)
        
        # /// Connect Main Street to Intersections
        intersectionRoadLM = self.addSubModel(IntersectionRoad(name="I_LM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["LM"])))
        intersectionRoadRM = self.addSubModel(IntersectionRoad(name="I_RM", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["RM"])))
        # IV. LM traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadLM.OUT_CAR_RIGHT, sectionCenter_MainLeft_LR.IN_CAR) 
        self.connectPorts(sectionCenter_MainRight_LR.OUT_CAR, intersectionRoadRM.IN_CAR_LEFT) 
        # V. Control
        # TODO

        # IV. RL Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadRM.OUT_CAR_LEFT, sectionCenter_MainRight_RL.IN_CAR) 
        self.connectPorts(sectionCenter_MainLeft_RL.OUT_CAR, intersectionRoadLM.IN_CAR_RIGHT) 
        # V. Control
        # TODO
        
        # /// Construct Left Street with Generator/Sink LM
        # IV. carGenLM and LR traffic
        carGeneratorLM = self.addSubModel(CarGeneratorModel(time_next_car=genAverageTime, identifier=1, pLocal=genPrecLocal, carDestinationMap = carGeneratorDestinations["LM"], useFixedTime=genUseFixedTime))
        sectionCenter_Left_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_L_LR", max_speed=50, length=lenCenter_L, initial_state=FLUID), "RoadC_L_LR"))
        
        # IV. Car connect
        self.connectPorts(carGeneratorLM.car_out, sectionCenter_Left_LR.IN_CAR)
        # V. Control connect
        self.connectPorts(sectionCenter_Left_LR.OUT_JAM, carGeneratorLM.IN_NEXT_JAM)

        # IV. CarSinkLM and RL traffic
        carSinkLM = self.addSubModel(CarSink(output_file='outfileLM.csv'))
        sectionCenter_Left_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_L_RL", max_speed=50, length=lenCenter_L, initial_state=FLUID), "RoadC_L_RL"))
        
        # V. Car Connect
        self.connectPorts(sectionCenter_Left_RL.OUT_CAR, carSinkLM.in_car)
        # V. Control Connect
        # Sink has none

        # /// Connect sectionCenter_Left To IntersectionLM
        # IV. Car Connect
        self.connectPorts(sectionCenter_Left_LR.OUT_CAR, intersectionRoadLM.IN_CAR_LEFT)
        self.connectPorts(intersectionRoadLM.OUT_CAR_LEFT, sectionCenter_Left_RL.IN_CAR)

        # IV. Control Connect
        # TODO

        
        # /// Construct Right Street with Generator/Sink RM
        # IV. carGenRM and RL-traffic
        carGeneratorRM = self.addSubModel(CarGeneratorModel(time_next_car=genAverageTime, identifier=2, pLocal=genPrecLocal, carDestinationMap = carGeneratorDestinations["RM"], useFixedTime=genUseFixedTime))
        sectionCenter_Right_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_R_RL", max_speed=50, length=lenCenter_R, initial_state=FLUID), "RoadC_R_RL"))
        
        # V. Car connect
        self.connectPorts(carGeneratorRM.car_out, sectionCenter_Right_RL.IN_CAR)
        # V. Control connect
        self.connectPorts(sectionCenter_Right_RL.OUT_JAM, carGeneratorRM.IN_NEXT_JAM)
        
        # IV. carSinkRM and LR-traffic
        carSinkRM = self.addSubModel(CarSink(output_file='outfileRM.csv'))
        sectionCenter_Right_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadC_L_LR", max_speed=50, length=lenCenter_R, initial_state=FLUID), "RoadC_L_LR"))
        
        # V. Car connect
        self.connectPorts(sectionCenter_Right_LR.OUT_CAR, carSinkRM.in_car)
        # V. Control connect
        # Sink has none

        # /// Connect Right To IntersectionRM
        # IV. Car Connect
        self.connectPorts(sectionCenter_Right_RL.OUT_CAR, intersectionRoadRM.IN_CAR_RIGHT)
        self.connectPorts(intersectionRoadRM.OUT_CAR_RIGHT, sectionCenter_Right_LR.IN_CAR)

        # IV. Control Connect
        # TODO

        # // Top Segment
        # /// Connect IntersectionLT and IntersectionRT horizontal
        # IV. Instantiate I_LT, I_RT, RoadT_H_LR, RoadT_H_RL
        intersectionRoadLT = self.addSubModel(IntersectionRoad(name="I_LT", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["LT"])))
        intersectionRoadRT = self.addSubModel(IntersectionRoad(name="I_RT", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["RT"])))
        sectionTop_H_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadT_H_LR", max_speed=50, length=lenTop_H, initial_state=FLUID), "RoadT_H_LR"))
        sectionTop_H_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadT_H_RL", max_speed=50, length=lenTop_H, initial_state=FLUID), "RoadT_H_RL"))

        # IV. LR Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadLT.OUT_CAR_RIGHT, sectionTop_H_LR.IN_CAR)
        self.connectPorts(sectionTop_H_LR.OUT_CAR, intersectionRoadRT.IN_CAR_LEFT)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadRT.OUT_CAR_LEFT, sectionTop_H_RL.IN_CAR)
        self.connectPorts(sectionTop_H_RL.OUT_CAR, intersectionRoadLT.IN_CAR_RIGHT)
        
        # V. Control Connect
        # TODO

        # /// Connect IntersectionLT and IntersectionRT top
        sectionTop_T_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadT_T_LR", max_speed=50, length=lenTop_T, initial_state=FLUID), "RoadT_T_LR"))
        sectionTop_T_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadT_T_RL", max_speed=50, length=lenTop_T, initial_state=FLUID), "RoadT_T_RL"))

        # IV. LR Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadLT.OUT_CAR_TOP, sectionTop_T_LR.IN_CAR)
        self.connectPorts(sectionTop_T_LR.OUT_CAR, intersectionRoadRT.IN_CAR_TOP)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadRT.OUT_CAR_TOP, sectionTop_T_RL.IN_CAR)
        self.connectPorts(sectionTop_T_RL.OUT_CAR, intersectionRoadLT.IN_CAR_TOP)
        
        # V. Control Connect
        # TODO

        # // Bot Segment
        # /// Connect IntersectionLB and Intersection RB horizontal
        # IV. Instantiate I_LB, I_RB, RoadB_H_LR, RoadB_H_RL
        intersectionRoadLB = self.addSubModel(IntersectionRoad(name="I_LB", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["LB"])))
        intersectionRoadRB = self.addSubModel(IntersectionRoad(name="I_RB", 
                                                             state=IntersectionRoadState(
                                                                 destinationMap = intersectionFilterMap["RB"])))
        sectionBot_H_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadB_H_LR", max_speed=50, length=lenBot_H, initial_state=FLUID), "RoadB_H_LR"))
        sectionBot_H_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadB_H_RL", max_speed=50, length=lenBot_H, initial_state=FLUID), "RoadB_H_RL"))

        # IV. LR Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadLB.OUT_CAR_RIGHT, sectionBot_H_LR.IN_CAR)
        self.connectPorts(sectionBot_H_LR.OUT_CAR, intersectionRoadRB.IN_CAR_LEFT)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadRB.OUT_CAR_LEFT, sectionBot_H_RL.IN_CAR)
        self.connectPorts(sectionBot_H_RL.OUT_CAR, intersectionRoadLB.IN_CAR_RIGHT)
        
        # V. Control Connect
        # TODO

        # /// Connect IntersectionLB and Intersection RB bot
        sectionBot_B_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadB_B_LR", max_speed=50, length=lenBot_B, initial_state=FLUID), "RoadB_B_LR"))
        sectionBot_B_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadB_B_RL", max_speed=50, length=lenBot_B, initial_state=FLUID), "RoadB_B_RL"))

        # IV. LR Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadLB.OUT_CAR_BOT, sectionBot_B_LR.IN_CAR)
        self.connectPorts(sectionBot_B_LR.OUT_CAR, intersectionRoadRB.IN_CAR_BOT)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic
        # V. Car Connect
        self.connectPorts(intersectionRoadRB.OUT_CAR_BOT, sectionBot_B_RL.IN_CAR)
        self.connectPorts(sectionBot_B_RL.OUT_CAR, intersectionRoadLB.IN_CAR_BOT)

        # V. Control Connect
        # TODO

        # / Vertical Segments
        # // Left side
        # /// Connect IntersectionLB with IntersectionLM
        sectionVert_LB_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_LB_LR", max_speed=50, length=lenVert_LB, initial_state=FLUID), "RoadV_LB_LR"))
        sectionVert_LB_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_LB_RL", max_speed=50, length=lenVert_LB, initial_state=FLUID), "RoadV_LB_RL"))
        

        # IV. LR Traffic : Mid to Bot
        # V. Car Connect
        self.connectPorts(intersectionRoadLM.OUT_CAR_BOT, sectionVert_LB_LR.IN_CAR)
        self.connectPorts(sectionVert_LB_LR.OUT_CAR, intersectionRoadLB.IN_CAR_TOP)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic : Bot to Mid
        # V. Car Connect
        self.connectPorts(intersectionRoadLB.OUT_CAR_TOP, sectionVert_LB_RL.IN_CAR)
        self.connectPorts(sectionVert_LB_RL.OUT_CAR, intersectionRoadLM.IN_CAR_BOT)

        # V. Control Connect
        # TODO


        # /// Connect IntersectionLT with IntersectionLM
        sectionVert_LT_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_LT_LR", max_speed=50, length=lenVert_LT, initial_state=FLUID), "RoadV_LT_LR"))
        sectionVert_LT_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_LT_RL", max_speed=50, length=lenVert_LT, initial_state=FLUID), "RoadV_LT_RL"))
        

        # IV. LR Traffic : Mid to Top
        # V. Car Connect
        self.connectPorts(intersectionRoadLM.OUT_CAR_TOP, sectionVert_LT_LR.IN_CAR)
        self.connectPorts(sectionVert_LT_LR.OUT_CAR, intersectionRoadLT.IN_CAR_BOT)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic : Top to Mid
        # V. Car Connect
        self.connectPorts(intersectionRoadLT.OUT_CAR_BOT, sectionVert_LT_RL.IN_CAR)
        self.connectPorts(sectionVert_LT_RL.OUT_CAR, intersectionRoadLM.IN_CAR_TOP)

        # V. Control Connect
        # TODO


        # // Right side
        # /// Connect IntersectionRB with IntersectionRM
        sectionVert_RB_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_RB_LR", max_speed=50, length=lenVert_RB, initial_state=FLUID), "RoadV_RB_LR"))
        sectionVert_RB_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_RB_RL", max_speed=50, length=lenVert_RB, initial_state=FLUID), "RoadV_RB_RL"))
        

        # IV. LR Traffic : Bot to Mid
        # V. Car Connect
        self.connectPorts(intersectionRoadRB.OUT_CAR_TOP, sectionVert_RB_LR.IN_CAR)
        self.connectPorts(sectionVert_RB_LR.OUT_CAR, intersectionRoadRM.IN_CAR_BOT)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic : Mid to Bot
        # V. Car Connect
        self.connectPorts(intersectionRoadRM.OUT_CAR_BOT, sectionVert_RB_RL.IN_CAR)
        self.connectPorts(sectionVert_RB_RL.OUT_CAR, intersectionRoadRB.IN_CAR_TOP)

        # V. Control Connect
        # TODO

        # /// Connect IntersectionRT with IntersectionRM
        sectionVert_RT_LR = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_RT_LR", max_speed=50, length=lenVert_RT, initial_state=FLUID), "RoadV_RT_LR"))
        sectionVert_RT_RL = self.addSubModel(
            RoadSectionModel(RoadSectionState(name="RoadV_RT_RL", max_speed=50, length=lenVert_RT, initial_state=FLUID), "RoadV_RT_RL"))
        

        # IV. LR Traffic : Top to Mid
        # V. Car Connect
        self.connectPorts(intersectionRoadRT.OUT_CAR_BOT, sectionVert_RT_LR.IN_CAR)
        self.connectPorts(sectionVert_RT_LR.OUT_CAR, intersectionRoadRM.IN_CAR_TOP)
        
        # V. Control Connect
        # TODO

        # IV. RL Traffic : Mid to Top
        # V. Car Connect
        self.connectPorts(intersectionRoadRM.OUT_CAR_TOP, sectionVert_RT_RL.IN_CAR)
        self.connectPorts(sectionVert_RT_RL.OUT_CAR, intersectionRoadRT.IN_CAR_BOT)

        # V. Control Connect
        # TODO