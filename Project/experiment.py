# Import code for model simulation, but using the minimal kernel:
from pypdevs.simulator import Simulator

# Import the model to be simulated
from Exercise_1 import SimpleRoadModel
#from Exercise_2 import SimpleRoadModel
#from Exercise_3 import SimpleRoadModel

#    ======================================================================

# 1. Instantiate the (Coupled or Atomic) DEVS at the root of the 
#  hierarchical model. This effectively instantiates the whole model 
#  thanks to the recursion in the DEVS model constructors (__init__).
#
trafficSystem = SimpleRoadModel()

#    ======================================================================

# 2. Link the model to a DEVS Simulator: 
#  i.e., create an instance of the 'Simulator' class,
#  using the model as a parameter.
sim = Simulator(trafficSystem)

#    ======================================================================

# 3. Perform all necessary configurations, with the minimal kernel, only setTerminationTime is supported.
#    e.g. to simulate until simulation time 400.0 is reached
sim.setTerminationTime(3000.0)

#    ======================================================================

# 4. Simulate the model
sim.setVerbose()
sim.setClassicDEVS()
sim.simulate()
