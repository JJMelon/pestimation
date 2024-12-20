from enum import Enum
from models.components.bus import Bus
from models.components.voltagesource import CurrentSensor
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
import numpy as np
from itertools import count

import numpy as np


class FuseStatus(Enum):
    GOOD = "GOOD"
    BLOWN = "BLOWN"


class Fuse():
    fuse_id_counter = count(0)

    def __init__(self,
                 from_node: Bus,
                 to_node: Bus,
                 interior_node: Bus,
                 current_limit,
                 status: FuseStatus,
                 phase
                 ):
        self.current_limit = current_limit
        self.status = status
        self.from_node = from_node
        self.to_node = to_node
        #self.interior_node = interior_node
        self.phase = phase
        self.G = 1e4
        self.B = 1e4
        # Adding for a fuse
        self.fuse_idx = next(self.fuse_id_counter)
        self.from_bus = from_node
        self.to_bus = to_node
        self.from_bus_idx = from_node.int_bus_id
        self.to_bus_idx = to_node.int_bus_id


    def get_connections(self):
        if self.status == FuseStatus.BLOWN:
            return []

        return [(self.from_node, self.to_node)]


    def get_current(self, v):
        if self.status == FuseStatus.BLOWN:
            raise Exception("No current available for blown fuse")

        return self.current_sensor.get_current(v)

    def try_adjust_device(self, v):
        if self.status == FuseStatus.BLOWN:
            # We assume that once a fuse is blown, we will never un-blow it.
            return False

        i_r, i_i = self.get_current(v)
        i = complex(i_r, i_i)
        if abs(i) > self.current_limit:
            self.status = FuseStatus.BLOWN
            return True

        return False

    ############## modification_starts #################

    def assign_ipopt_vars(self, model):
        self.ipopt_vr_from: Union[PyomoVar, float]
        self.ipopt_vi_from: Union[PyomoVar, float]
        self.ipopt_vr_to: Union[PyomoVar, float]
        self.ipopt_vi_to: Union[PyomoVar, float]

        self.ipopt_ir_fuse: Union[PyomoVar, float]
        self.ipopt_ii_fuse: Union[PyomoVar, float]

        self.ipopt_vr_from = model.ipopt_vr_list[self.from_bus_idx]
        self.ipopt_vi_from = model.ipopt_vi_list[self.from_bus_idx]
        self.ipopt_vr_to = model.ipopt_vr_list[self.to_bus_idx]
        self.ipopt_vi_to = model.ipopt_vi_list[self.to_bus_idx]
        ### Current from fuse ####
        self.ipopt_irf_from = model.fuse_ir_list[self.fuse_idx]
        self.ipopt_iif_from = model.fuse_ii_list[self.fuse_idx]

    def cal_fuse_data(self):
        Vr_fuse_constraint = (self.ipopt_vr_from - self.ipopt_vr_to)
        Vi_fuse_constraint = (self.ipopt_vi_from - self.ipopt_vi_to)
        Ir_fuse_from = self.ipopt_irf_from
        Ii_fuse_from = self.ipopt_iif_from

        Ir_fuse_to = -Ir_fuse_from
        Ii_fuse_to = -Ii_fuse_from

        return Vr_fuse_constraint, Vi_fuse_constraint, Ir_fuse_from, Ii_fuse_from, Ir_fuse_to, Ii_fuse_to

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag, vr_fuse_constraint, vi_fuse_constraint):
        # KCL terms for the fuse model
        (Vr_fuse_constraint, Vi_fuse_constraint,
         Ir_fuse_from, Ii_fuse_from,
         Ir_fuse_to, Ii_fuse_to) = self.cal_fuse_data()

        if self.from_bus_idx in KCL_equations_real:
            KCL_equations_real[self.from_bus_idx] += Ir_fuse_from
        else:
            KCL_equations_real[self.from_bus_idx] = Ir_fuse_from

        if self.from_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.from_bus_idx] += Ii_fuse_from
        else:
            KCL_equations_imag[self.from_bus_idx] = Ii_fuse_from

        if self.to_bus_idx in KCL_equations_real:
            KCL_equations_real[self.to_bus_idx] += Ir_fuse_to
        else:
            KCL_equations_real[self.to_bus_idx] = Ir_fuse_to

        if self.to_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.to_bus_idx] += Ii_fuse_to
        else:
            KCL_equations_imag[self.to_bus_idx] = Ii_fuse_to

        # voltage equation terms of the fuse model (it is modeled as a short-circuit)
        vr_fuse_constraint[self.fuse_idx] = Vr_fuse_constraint
        vi_fuse_constraint[self.fuse_idx] = Vi_fuse_constraint
        # vr_fuse_constraint[self.to_bus_idx] = -Vr_fuse_constraint
        # vi_fuse_constraint[self.to_bus_idx] = -Vi_fuse_constraint

    def initialize_ipopt_vars(self, model):
        model.fuse_ir_list[self.fuse_idx].value = 0.0
        model.fuse_ii_list[self.fuse_idx].value = 0.0
