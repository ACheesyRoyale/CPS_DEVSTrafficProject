from pypdevs.DEVS import CoupledDEVS
from lightCrossing import LightCrossingModel

class IntersectionLight(CoupledDEVS):
    def __init__(self, id, t_verticalRed, t_verticalYellow, t_verticalGreen):
        """
        A simple road consisting of 5 road sections :
            Section 1 & 5 : 70kph, 2500m,
            Section 2 & 4 : 50kph, 800m,
            Section 3 : 30 kph, 200m.
        """
        
        CoupledDEVS.__init__(self, "TF "+str(id))
        self.IN_NEXT_JAM_LR = self.addInPort("in_jam_control_lr")
        self.OUT_JAM_LR = self.addOutPort("out_jam_control_lr")
        self.IN_NEXT_JAM_RL= self.addInPort("in_jam_control_rl")
        self.OUT_JAM_RL = self.addOutPort("out_jam_control_rl")

        self.IN_NEXT_JAM_TB = self.addInPort("in_jam_control_tb")
        self.OUT_JAM_TB = self.addOutPort("out_jam_control_tb")
        self.IN_NEXT_JAM_BT = self.addInPort("in_jam_control_bt")
        self.OUT_JAM_BT = self.addOutPort("out_jam_control_bt")

        verticalLight = self.addSubModel(LightCrossingModel(t_green=140, t_yellow=30, t_red=100))
        horizontalLight = self.addSubModel(LightCrossingModel(t_green=140, t_yellow=30, t_red=100))

