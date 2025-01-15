from pypdevs.DEVS import AtomicDEVS
from pypdevs.infinity import INFINITY
import pandas as pd


class CarSink(AtomicDEVS):
    """
    DEVS atomic model that collects cars exiting the road system.
    """

    def __init__(self, output_file):
        AtomicDEVS.__init__(self, "TrafficSink")
        self.in_car = self.addInPort("incoming_car")
        self.exit_time = 0.0
        self.of = output_file

    def extTransition(self, inputs):
        print("carSink externalTransition")
        if self.in_car in inputs:
            #print("carSink externalTransition | in_car")
            car = inputs[self.in_car]
            self.exit_time += self.elapsed
            #print("carSink extTransition | exit_time", self.exit_time)
            car_time_in_system = self.exit_time - car.creation_time
            #print("carSink extTransition | Car exited.", car.identifier, car_time_in_system)
            
            # Write car data to csv file
            data = {'Id': [car.identifier], 'Exit Time': [self.exit_time], 'Distance': [car.distance_done], 'Travel Time': [car_time_in_system], 'local': [car.isLocal]}
            df = pd.DataFrame(data)
            df.to_csv(self.of, mode='a', index=False, header=False)

    def timeAdvance(self):
        return INFINITY
