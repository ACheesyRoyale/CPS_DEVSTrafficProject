from pypdevs.DEVS import CoupledDEVS


from carSink import CarSink
from carGenerator import CarGeneratorModel
from intersectionRoad import IntersectionRoad, IntersectionRoadState

class SimpleRoadModel(CoupledDEVS):
    def __init__(self):
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, "system")

        self.testIntersectionRoad()

    def testIntersectionRoad(self):
        carGeneratorR = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=1, pLocal=0.7))
        carSinkT = self.addSubModel(CarSink(output_file='outfile.csv'))
        carGeneratorL = self.addSubModel(CarGeneratorModel(time_next_car=100, identifier=2, pLocal=0.5))
        intersectionRoad = self.addSubModel(IntersectionRoad(name="test"))

        
        # Couple the car generator's output port to the road section's input port
        self.connectPorts(carGeneratorR.car_out, intersectionRoad.IN_CAR_RIGHT)
        self.connectPorts(carGeneratorL.car_out, intersectionRoad.IN_CAR_LEFT)
        
        # Couple exit to car sin
        self.connectPorts(intersectionRoad.OUT_CAR_TOP, carSinkT.in_car)