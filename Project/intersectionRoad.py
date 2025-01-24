from pypdevs.DEVS import *
from pypdevs.infinity import INFINITY
from random import choice as rnChoice

class IntersectionRoadState:
    def __init__(self, destinationMap = {}):
        self.dict_cars = {
            "T": [],
            "B":[],
            "R":[],
            "L":[]
        }
        self.carInIntersection = False
        self.destMap = destinationMap

    def calculate_time_advance(self):
        if self.carInIntersection:
            return 10 # small delay required to prevent collision. Is realistic.
        else:
            return INFINITY
        
    def output_car(self):
        dict_filteredCars = {}
        # output one car per direction
        for dir, car_list in self.dict_cars.items(): 
            if car_list:
                dict_filteredCars[dir] = car_list[0]
                car_list.pop(0)
            else:
                dict_filteredCars[dir] = None

        # cars left?
        self.carInIntersection = any(car_list for car_list in self.dict_cars.values())

        return dict_filteredCars
    
    def car_enters(self, dict_cars):
        self.carInIntersection = True
        # sort incoming cars per outgoing direction
        for incomingDirection, car in dict_cars.items():
            outgoingDirection = self.filterCars(incomingDirection, car)
            self.dict_cars[outgoingDirection].append(car)
    
    def filterCars(self, incomingDir, car):
        outDir = self.carFilter(incomingDir, car)
        return outDir
    
    def carFilter(self, incomingDir, car):
        print(car.destination)
        print(self.destMap)
        destinationDirections = self.destMap[car.destination]
        print(destinationDirections)
        localDirections = list(destinationDirections[0])
        nonLocalDirections = list(destinationDirections[1])
        if car.isLocal: 
            return rnChoice(localDirections)
        else:
            return rnChoice(nonLocalDirections)

    def sentAllNorth(self):
        return "T"

class IntersectionRoad(AtomicDEVS):
    """
        IntersectionRoad acts as filter between connected roads. Action should happen immediately.
            - Traffic Control is delegated to separated class (IntersectionTrafficLight).
        Filtering is based on 'local' or 'passing' attribute of incoming car.
    """
    def __init__(self, name, state):
        AtomicDEVS.__init__(self, name)
        
        if not isinstance(state, IntersectionRoadState):
            print("error in init of roadSectionModel")
            exit(1)

        self.state = state
        self.IN_CAR_TOP = self.addInPort("incoming_car_TOP")
        self.IN_CAR_BOT = self.addInPort("incoming_car_BOT")
        self.IN_CAR_RIGHT = self.addInPort("incoming_car_RIGHT")
        self.IN_CAR_LEFT = self.addInPort("incoming_car_LEFT")

        self.OUT_CAR_TOP = self.addOutPort("outgoing_car_TOP")
        self.OUT_CAR_BOT = self.addOutPort("outgoing_car_BOT")
        self.OUT_CAR_RIGHT = self.addOutPort("outgoing_car_RIGHT")
        self.OUT_CAR_LEFT = self.addOutPort("outgoing_car_LEFT")

    def timeAdvance(self):
        return self.state.calculate_time_advance()
    
    def extTransition(self, inputs):
        dict_car = {}
        if self.IN_CAR_TOP in inputs:
            dict_car["T"] = inputs[self.IN_CAR_TOP]
        if self.IN_CAR_BOT in inputs:
            dict_car["B"] = inputs[self.IN_CAR_BOT]
        if self.IN_CAR_RIGHT in inputs:
            dict_car["R"] = inputs[self.IN_CAR_RIGHT]
        if self.IN_CAR_LEFT in inputs:
            dict_car["L"] = inputs[self.IN_CAR_LEFT]
        if dict_car is not None:
            self.state.car_enters(dict_car)
        return self.state

    def outputFnc(self):
        """ Filter and output car"""
        dict_car = self.state.output_car()
        rv= {}
        if "T" in dict_car and dict_car["T"] is not None:
            rv[self.OUT_CAR_TOP] = dict_car["T"]
        if "B" in dict_car and dict_car["B"] is not None:
            rv[self.OUT_CAR_BOT] = dict_car["B"]
        if "R" in dict_car and dict_car["R"] is not None:
            rv[self.OUT_CAR_RIGHT] = dict_car["R"]
        if "L" in dict_car and dict_car["L"] is not None:
            rv[self.OUT_CAR_LEFT] = dict_car["L"]
        
        return rv