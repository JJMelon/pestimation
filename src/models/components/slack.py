from sympy import symbols
import math
import numpy as np
from models.components.bus import GROUND, Bus
from models.wellknownvariables import Vr_from, Vi_from, Vr_to, Vi_to
from pyomo.environ import Var, ConcreteModel, ConstraintList, Objective
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
from models.components.bus import Bus
from itertools import count


class Slack:
    slack2index_dict = {}
    slack_id_counter = count(0)
    slack2index_dict = {}

    def __init__(self,
                 bus: Bus,
                 Vset,
                 ang,
                 Pinit,
                 Qinit):
        """Initialize slack bus in the power grid.

        Args:
            Bus (int): the bus number corresponding to the slack bus.
            Vset (float): the voltage setpoint that the slack bus must remain fixed at.
            ang (float): the slack bus voltage angle that it remains fixed at.
            Pinit (float): the initial active power that the slack bus is supplying
            Qinit (float): the initial reactive power that the slack bus is supplying
        """
        self.bus = bus
        self.bus.Type = 0
        self.Vset = Vset
        self.ang_rad = ang
        self.Vr_set = self.Vset * math.cos(self.ang_rad)
        self.Vi_set = self.Vset * math.sin(self.ang_rad)
        self.Pinit = Pinit / 100
        self.Qinit = Qinit / 100
        self.slack_idx = next(self.slack_id_counter)
        self.slack_bus_id = bus.int_bus_id

    def slack_node_info(self):
        return self.slack_bus_id

    def get_connections(self):
        return [(self.bus, GROUND)]
    # Added function to get the slack variables

    def assign_ipopt_vars(self, model):
        self.ipopt_vr:Union[PyomoVar, float]
        self.ipopt_vi:Union[PyomoVar, float]
        self.ipopt_ir_slack:Union[PyomoVar, float]
        self.ipopt_ii_slack:Union[PyomoVar, float]
        ##### Infeasibility variables ######
        self.ipopt_ifeas_ir:Union[PyomoVar, float]
        self.ipopt_infeas_ii:Union[PyomoVar, float]
        #self.ipopt_ir_slack = model.slack_vr_list[PyomoVar, float]
        #self.ipopt_ii_slack = model.slack_vi_list[PyomoVar, float]
        # Index of the node to which the slack voltage is connected
        self.ipopt_vr = model.ipopt_vr_list[self.bus.NodeFullName] # Ipopt_vr_list already a list ---> just passing the self.bus.bus_id to get the list
        self.ipopt_vi = model.ipopt_vi_list[self.bus.NodeFullName] # Ipopt_vi_list already a list ---> just passing the self.bus.bus_id to get the list

        # Setting the slack infeasibility current to zero
        #### Commenting it out for a second ####
        # self.ipopt_ifeas_ir =  model.infeasi_ir_list[self.bus.bus_id]
        # self.ipopt_infeas_ii = model.infeasi_ii_list[self.bus.bus_id]
        ##### For slack infeasibility  ######

         # Index for where to write the voltage equations
        self.ipopt_ir_slack = model.slack_vr_list[self.bus.NodeFullName]
        self.ipopt_ii_slack = model.slack_vi_list[self.bus.NodeFullName]

        #### Setting it to zero; not a better approach ####
        # self.ipopt_ir_slack.value = 0.0
        # self.ipopt_ii_slack.value = 0.0


    def cal_slack_data(self):
        Vr_slack_constraint = self.ipopt_vr - self.Vr_set
        Vi_slack_constraint = self.ipopt_vi - self.Vi_set
        Ir_slack = self.ipopt_ir_slack
        Ii_slack = self.ipopt_ii_slack

        ##### slack_infeasibility
        # Ir_infeasi_slack = self.ipopt_ifeas_ir
        # Ii_infeasi_slack = self.ipopt_infeas_ii

        return Vr_slack_constraint, Vi_slack_constraint, Ir_slack, Ii_slack

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag, vr_slack_constraint, vi_slack_constraint):
        (Vr_slack_constraint, Vi_slack_constraint, Ir_slack, Ii_slack) = self.cal_slack_data()

        # Adding the current contributions of the slack bus to the KCL equations
        if self.bus.int_bus_id in KCL_equations_real:
            KCL_equations_real[self.bus.int_bus_id] += Ir_slack
        else:
            KCL_equations_real[self.bus.int_bus_id] = Ir_slack

        if self.bus.int_bus_id in KCL_equations_imag:
            KCL_equations_imag[self.bus.int_bus_id] += Ii_slack
        else:
            KCL_equations_imag[self.bus.int_bus_id] = Ii_slack

        # Adding the voltage constraints for the slack bus
        vr_slack_constraint[self.bus.int_bus_id] = Vr_slack_constraint
        vi_slack_constraint[self.bus.int_bus_id] = Vi_slack_constraint






