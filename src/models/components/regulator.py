from itertools import count
from enum import Enum
from models.components.bus import GROUND, Bus
from models.components.transformer import Transformer
from models.components.voltagesource import CurrentSensor
from pyomo.environ import Var as PyomoVar, value
import math
from typing import Union

class RegType(Enum):
    A = "A"
    B = "B"

class RegControl(Enum):
    MANUAL = "MANUAL"
    REMOTE_NODE = "REMOTE_NODE"
    OUTPUT_VOLTAGE = "OUTPUT_VOLTAGE"
    LINE_DROP_COMP = "LINE_DROP_COMP"

class Regulator():
    _ids = count(0)

    def __init__(self
                , from_node: Bus 
                , to_node: Bus
                # , current_node: Bus
                , tap_position: int
                , ar_step
                , reg_type: RegType
                , reg_control: RegControl
                , vlow: float
                , vhigh: float
                , raise_taps: int
                , lower_taps: int
                ):
        self.id = self._ids.__next__()
        self.from_node = from_node
        self.to_node = to_node
        # self.current_node = current_node
        self.tap_position = tap_position        
        self.ar_step = ar_step
        self.reg_type = reg_type
        self.reg_control = reg_control
        self.vlow = vlow
        self.vhigh = vhigh
        self.raise_taps = raise_taps
        self.lower_taps = lower_taps
        self.name = "Regulator"


        #this is temporary before we update the tap position for the first time.
        self.turn_ratio = 0

        #todo: is this correct?
        self.phase_shift = 0 

        #todo: pull better values?
        r = 1e-4
        x = 0
        self.r=r
        self.x=x
        self.g = r / (r ** 2 + x ** 2)
        self.b = -x / (r ** 2 + x ** 2)
        
        self.transformer = Transformer(
            self.name,
            self.from_node, 
            GROUND, 
            self.to_node, 
            GROUND,
            r,
            x,
            True,
            self.turn_ratio,
            self.phase_shift,
            0,
            0,
            None,
            None,
            None,
            is_regulator=True  # Indicate that this is part of a regulator
            )

        #Some control modes require a current measurement on the output of the reg.
        # self.current_sensor = CurrentSensor(self.current_node, self.to_node)

        self.try_increment_tap_position(0)

    def try_increment_tap_position(self, increment):
        old_position = self.tap_position
        if self.tap_position + increment >= self.raise_taps:
            self.tap_position = self.raise_taps
        elif self.tap_position + increment <= -self.lower_taps:
            self.tap_position = -self.lower_taps
        else:
            self.tap_position += increment

        if self.reg_type == RegType.A:
            self.turn_ratio = (1 + (self.ar_step * self.tap_position)) ** -1
        elif self.reg_type == RegType.B:
            self.turn_ratio = 1 - (self.ar_step * self.tap_position)
        else:
            raise Exception(f"Unknown regulator type {self.reg_type}")
        
        self.transformer.tr = self.turn_ratio

        return old_position != self.tap_position
        

    def try_adjust_device(self, v):
        #Still a work in progress. Disabling for now.
        return False

        adjustment_made = False
        if self.reg_control == RegControl.MANUAL:
            return False
        elif self.reg_control == RegControl.OUTPUT_VOLTAGE:
            v_r, v_i = v[self.to_node.node_Vr], v[self.to_node.node_Vi]
            v_mag = abs(complex(v_r, v_i))

            if v_mag < self.vlow:
                if self.try_increment_tap_position(1):
                    adjustment_made = True
            elif v_mag > self.vhigh:
                if self.try_increment_tap_position(-1):
                    adjustment_made = True
        else:
            raise Exception(f"{self.reg_control} mode for regulator not implemented")
    
        return adjustment_made

    def assign_nodes(self, node_index, optimization_enabled):
        self.transformer.assign_nodes(node_index, optimization_enabled)
        self.current_sensor.assign_nodes(node_index, optimization_enabled)

    def get_stamps(self):
        return self.transformer.get_stamps() + self.current_sensor.get_stamps()

    def get_connections(self):
        return [(self.from_node, self.to_node)]
    
    def assign_reg_ipopt_vars(self, model):
        self.ipopt_ir_pri_reg: Union[PyomoVar, float]
        self.ipopt_ii_pri_reg: Union[PyomoVar, float]
        self.ipopt_vr_sec_reg: Union[PyomoVar, float]
        self.ipopt_vi_sec_reg: Union[PyomoVar, float]
        self.ipopt_vr_pri_reg: Union[PyomoVar, float]
        self.ipopt_vi_pri_reg: Union[PyomoVar, float]
        self.ipopt_vr_aux_sec_reg: Union[PyomoVar, float]
        self.ipopt_vi_aux_sec_reg: Union[PyomoVar, float]
        
        self.ipopt_vr_aux_sec_reg = model.reg_vr_aux_list[self.id] # This is the aux node on the secondary
        self.ipopt_vi_aux_sec_reg = model.reg_vi_aux_list[self.id]
        self.ipopt_ir_pri_reg = model.reg_ir_pri_list[self.id] # The current on the primary voltage source
        self.ipopt_ii_pri_reg = model.reg_ii_pri_list[self.id]
        self.ipopt_vr_pri_reg = model.ipopt_vr_list[self.from_node.int_bus_id] # The from node to which the primary voltage source is connected
        self.ipopt_vi_pri_reg = model.ipopt_vi_list[self.from_node.int_bus_id]
        self.ipopt_vr_sec_reg = model.ipopt_vr_list[self.to_node.int_bus_id] # To node connected to aux node via impedance
        self.ipopt_vi_sec_reg = model.ipopt_vi_list[self.to_node.int_bus_id]


    def calc_reg_data(self):
        # Assuming there is no angle shift
        Ir_prim_reg = self.ipopt_ir_pri_reg
        Ii_prim_reg = self.ipopt_ii_pri_reg
        ir_sec_reg = ((self.ipopt_vr_sec_reg - self.ipopt_vr_aux_sec_reg) * self.g - self.b * (self.ipopt_vi_sec_reg - self.ipopt_vi_aux_sec_reg))
        ii_sec_reg = ((self.ipopt_vi_sec_reg - self.ipopt_vi_aux_sec_reg) * self.g + self.b * (self.ipopt_vr_sec_reg - self.ipopt_vr_aux_sec_reg))
        ir_aux_sec_reg = -ir_sec_reg - self.ipopt_ir_pri_reg * self.transformer.tr
        ii_aux_sec_reg = -ii_sec_reg - self.ipopt_ii_pri_reg * self.transformer.tr
        V_r_prim_cons_reg = self.ipopt_vr_pri_reg - self.ipopt_vr_aux_sec_reg * self.transformer.tr
        V_i_prim_cons_reg = self.ipopt_vi_pri_reg - self.ipopt_vi_aux_sec_reg * self.transformer.tr

        return (Ir_prim_reg, Ii_prim_reg, ir_aux_sec_reg, ii_aux_sec_reg, V_r_prim_cons_reg, V_i_prim_cons_reg, ir_sec_reg, ii_sec_reg)

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag, reg_vr_aux_constraint, reg_vi_aux_constraint, reg_vr_pri_constraint, reg_vi_pri_constraint):
        (Ir_prim_reg, Ii_prim_reg, ir_aux_sec_reg, ii_aux_sec_reg, V_r_prim_cons_reg, V_i_prim_cons_reg, ir_sec_reg, ii_sec_reg) = self.calc_reg_data()
        
        # Primary side (from node) and Secondary side (to node)
        self.update_kcl(KCL_equations_real, KCL_equations_imag, self.from_node.int_bus_id, Ir_prim_reg, Ii_prim_reg)
        self.update_kcl(KCL_equations_real, KCL_equations_imag, self.to_node.int_bus_id, ir_sec_reg, ii_sec_reg)
        
        # # Print statements to debug constraints
        # print(f"Adding constraints for Regulator {self.id}:")
        # print(f"Voltage Auxiliary Real Constraint at {self.to_node.int_bus_id}: {ir_aux_sec_reg}")
        # print(f"Voltage Auxiliary Imag Constraint at {self.to_node.int_bus_id}: {ii_aux_sec_reg}")
        # print(f"Voltage Primary Real Constraint at {self.id}: {V_r_prim_cons_reg}")
        # print(f"Voltage Primary Imag Constraint at {self.id}: {V_i_prim_cons_reg}")

        # Voltage Constraints
        reg_vr_aux_constraint[self.to_node.int_bus_id] = ir_aux_sec_reg
        reg_vi_aux_constraint[self.to_node.int_bus_id] = ii_aux_sec_reg
        reg_vr_pri_constraint[self.id] = V_r_prim_cons_reg
        reg_vi_pri_constraint[self.id] = V_i_prim_cons_reg

    def update_kcl(self, KCL_equations_real, KCL_equations_imag, int_bus_id, Ir, Ii):
        # Updating KCL equations for a given bus
        if int_bus_id in KCL_equations_real:
            KCL_equations_real[int_bus_id] += Ir
        else:
            KCL_equations_real[int_bus_id] = Ir

        if int_bus_id in KCL_equations_imag:
            KCL_equations_imag[int_bus_id] += Ii
        else:
            KCL_equations_imag[int_bus_id] = Ii