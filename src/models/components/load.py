from itertools import count
import numpy as np
from sympy import symbols
from models.components.bus import Bus
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
from pyomo.environ import Var, ConcreteModel, ConstraintList, Objective
from models.wellknownvariables import Vr_from, Vi_from, Vr_to, Vi_to, Lr_from, Li_from, Lr_to, Li_to
import random
import math

# Eqns reference:
# Pandey, A. (2018).
# Robust Steady-State Analysis of Power Grid using Equivalent Circuit Formulation with Circuit Simulation Methods

# Represents a two-terminal load. Can be used for positive sequence or three phase.

class Load:
    _ids = count(0)
    net_solar = 0
    net_load = 0
    counter = 0
    total_P = 0
    total_Q = 0
    def __init__(self,
                 name,
                 from_bus: Bus,
                 to_bus: Bus,
                 P,
                 Q,
                 Z,
                 IP,
                 IQ,
                 load_num=None,
                 phase=None,
                 triplex_phase=None
                 ):
        """Initialize an instance of a PQ or ZIP load in the power grid.

        Args:
            Bus (int): the bus where the load is located
            P (float): the active power of a constant power (PQ) load.
            Q (float): the reactive power of a constant power (PQ) load.
            Z (complex): the linear impedance of the load.
            IP (float): the active power component of a constant current load.
            IQ (float): the reactive power component of a constant current load.
        """

        self.id = Load._ids.__next__()
        self.name = name
        self.load_num = load_num if load_num != None else str(self.id)
        self.triplex_phase = triplex_phase
        self.phase = phase
        if triplex_phase:
            self.phase_int = int(phase)
        else:
            self.phase_int = None
        self.from_bus = from_bus
        self.to_bus = to_bus
        self.P = P
        self.Q = Q
        if self.triplex_phase:
            self.P = self.P *-1
        self.Z = Z
        self.IP = IP
        self.IQ = IQ
        self.from_bus_idx = from_bus.int_bus_id
        # print("******************************************************************")
        # print(self.from_bus_idx)
        # print("******************************************************************")
        if self.phase_int == 12:
            self.to_bus_idx = to_bus.int_bus_id
        if not self.Z == 0:
            r = np.real(self.Z)
            x = np.imag(self.Z)
            self.G = r / (r**2 + x**2)
            self.B = -x / (r**2 + x**2)
        else:
            self.G = 0
            self.B = 0
        self.ipopt_vr = None
        self.ipopt_vi = None
        Load.net_load += np.sqrt(self.P**2 + self.Q**2)

    def get_connections(self):
        return [(self.from_bus, self.to_bus)]
    # Getting the information about the load

    def get_P(self):
        return self.P

    def get_Q(self):
        return self.Q

    def get_node_index(self):
        return self.phase

    def create_ipopt_vars(self, model):
        self.ipopt_vr: Union[PyomoVar, float]
        self.ipopt_vi: Union[PyomoVar, float]
        self.ipopt_infeas_ir: Union[PyomoVar, float]
        self.ipopt_infeas_ii: Union[PyomoVar, float]
        self.ipopt_vr = model.ipopt_vr_list[self.from_bus.NodeFullName]
        self.ipopt_vi = model.ipopt_vi_list[self.from_bus.NodeFullName]

    def cal_load_current(self):
        I_lr = ((self.P * self.ipopt_vr) + (self.Q * self.ipopt_vi)) / \
            (self.ipopt_vr**2 + self.ipopt_vi**2)
        I_li = ((self.P * self.ipopt_vi) - (self.Q * self.ipopt_vr)) / \
            (self.ipopt_vr**2 + self.ipopt_vi**2)
        return I_lr, I_li

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag):
        (I_lr, I_li) = self.cal_load_current()
        if self.from_bus_idx in KCL_equations_real:
            KCL_equations_real[self.from_bus_idx] += I_lr
        else:
            KCL_equations_real[self.from_bus_idx] = I_lr
        if self.from_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.from_bus_idx] += I_li
        else:
            KCL_equations_imag[self.from_bus_idx] = I_li


    def create_ipopt_vars_triplex(self, model):
            self.ipopt_vr_1: Union[PyomoVar, float]
            self.ipopt_vi_1: Union[PyomoVar, float]
            self.ipopt_vr_2: Union[PyomoVar, float]
            self.ipopt_vi_2: Union[PyomoVar, float]

            self.ipopt_vr_1 = model.ipopt_vr_list[self.from_bus.NodeFullName]
            self.ipopt_vi_1 = model.ipopt_vi_list[self.from_bus.NodeFullName]
            if '12' in self.phase:
                self.ipopt_vr_2 = model.ipopt_vr_list[self.to_bus.NodeFullName]
                self.ipopt_vi_2 = model.ipopt_vi_list[self.to_bus.NodeFullName]

    def cal_load_current_triplex(self):
        """Calculate current based on phase configuration."""
        currents = {}
        if '1' in self.phase:
            I_lr_1, I_li_1 = self.calc_current(self.P, self.Q, self.ipopt_vr_1, self.ipopt_vi_1)
            currents['1'] = (I_lr_1, I_li_1)
        if '2' in self.phase:
            I_lr_2, I_li_2 = self.calc_current(self.P, self.Q, self.ipopt_vr_1, self.ipopt_vi_1)
            currents['2'] = (I_lr_2, I_li_2)
        if '12' in self.phase:
            I_lr_12, I_li_12 = self.calc_current(self.P, self.Q, self.ipopt_vr_1 - self.ipopt_vr_2, self.ipopt_vi_1 - self.ipopt_vi_2)
            currents['12'] = (I_lr_12, I_li_12)
        return currents

    def calc_current(self, P, Q, vr, vi):
        I_lr = (P * vr + Q * vi) / (vr**2 + vi**2)
        I_li = (P * vi - Q * vr) / (vr**2 + vi**2)
        return I_lr, I_li

    def add_to_eqn_list_triplex(self, KCL_equations_real, KCL_equations_imag):
        """Update the KCL equations based on the calculated currents for each configuration."""
        currents = self.cal_load_current_triplex()
        for phase, (I_lr, I_li) in currents.items():
            if phase == '1':
                KCL_equations_real[self.from_bus_idx] += I_lr
                KCL_equations_imag[self.from_bus_idx] += I_li
            if phase == '2':
                KCL_equations_real[self.from_bus_idx] -= I_lr
                KCL_equations_imag[self.from_bus_idx] -= I_li
            if phase == '12':
                KCL_equations_real[self.from_bus_idx] += I_lr
                KCL_equations_imag[self.from_bus_idx] += I_li
                KCL_equations_real[self.to_bus_idx] -= I_lr
                KCL_equations_imag[self.to_bus_idx] -= I_li

    def print_load_current(self):
        Load.counter += 1
        I_lr = (self.P * self.ipopt_vr_1.value + self.Q * self.ipopt_vi_1.value) / (self.ipopt_vr_1.value**2 + self.ipopt_vi_1.value**2)
        I_li = (self.P * self.ipopt_vi_1.value - self.Q * self.ipopt_vr_1.value) / (self.ipopt_vr_1.value**2 + self.ipopt_vi_1.value**2)
        Imag_load = np.sqrt(I_lr**2 + I_li**2)
        Vmag_load = np.sqrt(self.ipopt_vr_1.value**2 + self.ipopt_vi_1.value**2)
        Imag_ang = math.degrees(math.atan2(I_li, I_lr))
        Vmag_ang = math.degrees(math.atan2(self.ipopt_vi_1.value, self.ipopt_vr_1.value))
        print(Load.counter, self.name, self.phase, np.sqrt(self.P**2 + self.Q**2), Vmag_load, Imag_load, Vmag_load*Imag_load)
