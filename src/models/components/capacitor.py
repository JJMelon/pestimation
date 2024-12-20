from enum import Enum
from models.components.bus import Bus
from pyomo.environ import Var, ConcreteModel, ConstraintList, Objective
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
import numpy as np

class CapSwitchState(Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"

class CapacitorMode(Enum):
    MANUAL = "MANUAL"
    VOLT = "VOLT"
    VAR = "VAR"
    VARVOLT = "VARVOLT"

class Capacitor:
    def __init__(self, from_bus: Bus, to_bus: Bus, var, nominal_voltage, mode: CapacitorMode, high_voltage, low_voltage) -> None:
        self.from_bus = from_bus
        self.to_bus = to_bus
        self.var = var
        self.mode = mode
        self.high_voltage = high_voltage
        self.low_voltage = low_voltage
        self.from_bus_idx = from_bus.int_bus_id

        self.G = 0
        #https://github.com/gridlab-d/gridlab-d/blob/62dec057ab340ac100c4ae38a47b7400da975156/powerflow/capacitor.cpp#L316
        self.B = var / (nominal_voltage * nominal_voltage)

        #we expect this to be set as part of device control
        self.switch: CapSwitchState
        self.switch = CapSwitchState.CLOSED

    def get_connections(self):
        return [(self.from_bus, self.to_bus)]

    def try_adjust_device(self, v):
        adjustment_made = False
        if self.mode == CapacitorMode.MANUAL:
            return False
        elif self.mode == CapacitorMode.VOLT:
            f_r, f_i = (self.from_bus.node_Vr, self.from_bus.node_Vi)
            v_r = v[f_r]
            v_i = v[f_i]

            v_magnitude = abs(complex(v_r,v_i))
            if v_magnitude > self.high_voltage:
                if self.switch == CapSwitchState.OPEN:
                    self.switch = CapSwitchState.CLOSED
                    adjustment_made = True
            if v_magnitude < self.low_voltage:
                if self.switch == CapSwitchState.CLOSED:
                    self.switch = CapSwitchState.OPEN
                    adjustment_made = True
        else:
            raise Exception(f"{self.mode} mode for capacitor not implemented")

        return adjustment_made
    def create_ipopt_vars(self, model):
        self.ipopt_vr: Union[PyomoVar, np.array]
        self.ipopt_vi: Union[PyomoVar, np.array]
        self.ipopt_vr = model.ipopt_vr_list[self.from_bus_idx]
        self.ipopt_vi = model.ipopt_vi_list[self.from_bus_idx]

    def calc_cap_current(self):
        I_cr = -(self.ipopt_vi)*self.B
        I_ci = (self.ipopt_vr)*self.B
        return I_cr, I_ci

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag):
        if self.switch == CapSwitchState.OPEN:
            return

        # Calculate the capacitor currents
        I_cr, I_ci = self.calc_cap_current()

        # Adding the real component of the capacitor current to the KCL equation
        if self.from_bus_idx in KCL_equations_real:
            KCL_equations_real[self.from_bus_idx] += I_cr
        else:
            KCL_equations_real[self.from_bus_idx] = I_cr

        # Adding the imaginary component of the capacitor current to the KCL equation

        if self.from_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.from_bus_idx] += I_ci
        else:
            KCL_equations_imag[self.from_bus_idx] = I_ci

