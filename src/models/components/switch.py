from enum import Enum
from models.components.bus import Bus
from models.components.voltagesource import VoltageSource
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
import numpy as np
from itertools import count


class SwitchStatus(Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"


class Switch():
    switch_id_counter = count(0)

    def __init__(self,
                 from_node: Bus,
                 to_node: Bus,
                 status: SwitchStatus,
                 phase):
        self.status = status
        self.from_node = from_node
        self.to_node = to_node
        self.phase = phase
        # Adding a switch
        self.switch_idx = next(self.switch_id_counter)
        self.from_bus = from_node
        self.to_bus = to_node
        self.from_bus_idx = from_node.int_bus_id
        self.to_bus_idx = to_node.int_bus_id
        self.vs = VoltageSource(from_node, to_node, 0, 0)

    def assign_nodes(self, node_index, optimization_enabled):
        if self.status == SwitchStatus.OPEN:
            return
        self.vs.assign_nodes(node_index, optimization_enabled)

    def get_stamps(self):
        if self.status == SwitchStatus.OPEN:
            return []
        return self.vs.get_stamps()

    def get_connections(self):
        if self.status == SwitchStatus.OPEN:
            return []
        return [(self.from_node, self.to_node)]

    ############## modification_starts #################
    def assign_ipopt_vars(self, model):
        self.ipopt_vr_from: Union[PyomoVar, float]
        self.ipopt_vi_from: Union[PyomoVar, float]
        self.ipopt_vr_to: Union[PyomoVar, float]
        self.ipopt_vi_to: Union[PyomoVar, float]
        self.ipopt_ir_switch: Union[PyomoVar, float]
        self.ipopt_ii_switch: Union[PyomoVar, float]

        self.ipopt_vr_from = model.ipopt_vr_list[self.from_bus.NodeFullName]
        self.ipopt_vi_from = model.ipopt_vi_list[self.from_bus.NodeFullName]
        self.ipopt_vr_to = model.ipopt_vr_list[self.to_bus.NodeFullName]
        self.ipopt_vi_to = model.ipopt_vi_list[self.to_bus.NodeFullName]

        ### Current from switch ####
        self.ipopt_irs_from = model.switch_ir_list[self.switch_idx]
        self.ipopt_iis_from = model.switch_ii_list[self.switch_idx]

    def cal_switch_data(self):
        Vr_switch_constraint = (self.ipopt_vr_from - self.ipopt_vr_to)
        Vi_switch_constraint = (self.ipopt_vi_from - self.ipopt_vi_to)
        Ir_switch_from = self.ipopt_irs_from
        Ii_switch_from = self.ipopt_iis_from
        Ir_switch_to = -Ir_switch_from
        Ii_switch_to = -Ii_switch_from

        return (Vr_switch_constraint, Vi_switch_constraint, Ir_switch_from, \
            Ii_switch_from, Ir_switch_to, Ii_switch_to)

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag,
                        vr_switch_constraint, vi_switch_constraint):
        # KCL terms for the switch model
        (Vr_switch_constraint, Vi_switch_constraint,
         Ir_switch_from, Ii_switch_from,
         Ir_switch_to, Ii_switch_to) = self.cal_switch_data()

        if self.from_bus_idx in KCL_equations_real:
            KCL_equations_real[self.from_bus_idx] += Ir_switch_from
        else:
            KCL_equations_real[self.from_bus_idx] = Ir_switch_from

        if self.from_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.from_bus_idx] += Ii_switch_from
        else:
            KCL_equations_imag[self.from_bus_idx] = Ii_switch_from

        if self.to_bus_idx in KCL_equations_real:
            KCL_equations_real[self.to_bus_idx] += Ir_switch_to
        else:
            KCL_equations_real[self.to_bus_idx] = Ir_switch_to

        if self.to_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.to_bus_idx] += Ii_switch_to
        else:
            KCL_equations_imag[self.to_bus_idx] = Ii_switch_to

        # voltage equation terms of the switch model (it is modeled as a short-circuit)
        vr_switch_constraint[self.switch_idx] = Vr_switch_constraint
        vi_switch_constraint[self.switch_idx] = Vi_switch_constraint
        # vr_switch_constraint[self.to_bus_idx] = -Vr_switch_constraint
        # vi_switch_constraint[self.to_bus_idx] = -Vi_switch_constraint

    def initialize_ipopt_vars(self, model):
        model.switch_ir_list[self.switch_idx].value = 0.0
        model.switch_ii_list[self.switch_idx].value = 0.0
