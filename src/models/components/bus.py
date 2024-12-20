from __future__ import division
import math
from pyomo.environ import Var, ConcreteModel, ConstraintList, Objective
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
from itertools import count

#Represents an interconnection point for other network element with a shared voltage. In the three-phase case, this is used for a single phase.
class Bus:

    bus2index_dict = {}
    bus_id_counter = count(0)

    def __init__(self,
                 Bus,
                 Type,
                 Vm_init,
                 Va_init,
                 Area,
                 NodeName = None,
                 NodeParent = None,
                 NodePhase = None,
                 IsVirtual = False):
        """
        Args:
            Bus (int): The bus number.
            Type (int): The type of bus (e.g., PV, PQ, of Slack)
            Vm_init (float): The initial voltage magnitude at the bus.
            Va_init (float): The initial voltage angle at the bus **in radians**
            Area (int): The zone that the bus is in.
        """

        self.bus = Bus
        self.Type = Type
        self.NodeName = NodeName if NodeName is not None else f"Bus:{self.Bus}"
        self.NodeParent = NodeParent
        self.NodePhase = NodePhase if NodePhase is not None else f"NA"
        self.NodeFullName = self.NodeName + ':' + self.NodePhase
        self.IsVirtual = IsVirtual
        self.V_Nominal = Vm_init
        self.Va_init = Va_init
        self.Vmax = 1.1 * self.V_Nominal
        self.Vmin = 0.9 * self.V_Nominal
        # initialize all nodes
        self.node_Vr: int # real voltage node at a bus
        self.node_Vi: int # imaginary voltage node at a bus

        self.Vr_init = Vm_init * math.cos(Va_init)
        self.Vi_init = Vm_init * math.sin(Va_init)
        #print("Vr_init: ", self.Vr_init)
        #print("Vi_init: ", self.Vi_init)
        # Create a data structure to store the bus id
        # initialize the bus key
        if self.NodeName != "Gnd":
            self.int_bus_id = self.bus_id_counter.__next__()
            self.bus2index_dict[self.bus] = self.int_bus_id

    def create_ipopt_bus_vars(self, model):

        self.ipopt_vr:Union[PyomoVar, float]
        self.ipopt_vi:Union[PyomoVar, float]

        self.ipopt_vr = model.ipopt_vr_list[self.NodeFullName]
        self.ipopt_vi = model.ipopt_vi_list[self.NodeFullName]
        self.ipopt_vr.value = self.Vr_init
        self.ipopt_vi.value = self.Vi_init


    def __hash__(self) -> int:
        return self.bus

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, Bus):
            return __o.Bus == self.Bus
        else:
            return False

    def set_initial_voltages(self, v_estimate):
        f_r, f_i = (self.node_Vr, self.node_Vi)
        v_estimate[f_r] = self.Vr_init if v_estimate[f_r] == 0 else v_estimate[f_r]
        v_estimate[f_i] = self.Vi_init if v_estimate[f_i] == 0 else v_estimate[f_i]

    def __str__(self):
        return f'Bus: {self.bus} (Vr:{self.node_Vr} Vi:{self.node_Vi})'

    def __repr__(self):
        return f'Bus {self.bus} ({self.NodeFullName}) Vr:{self.node_Vr} Vi:{self.node_Vi}'

    def set_Vnominal(self, Vnom):
        self.V_Nominal = Vnom

    def apply_voltage_limit(self, model):
        if not self.IsVirtual:
            V_squared = self.ipopt_vr**2 + self.ipopt_vi**2
            model.cons.add(V_squared <= self.Vmax**2)
            model.cons.add(V_squared >= self.Vmin**2)


    def get_connections(self):
        return []

GROUND = Bus(None, "Gnd", 0, 0, None, "Gnd", "Gnd")
