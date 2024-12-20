from itertools import count
import math
from models.components.bus import Bus
from typing import Union
from pyomo.environ import Var as PyomoVar

class Transformer:
    _ids = count(0)

    def __init__(self,
                 name,
                 from_bus_pos: Bus,
                 from_bus_neg: Bus,
                 to_bus_pos: Bus,
                 to_bus_neg: Bus,
                 r,
                 x,
                 status,
                 tr,
                 ang,
                 G_shunt,
                 B_shunt,
                 rating,
                 primary_coil,
                 secondary_coil,
                 is_regulator=False):
        # ID setup
        if not is_regulator:
            self.id = next(self._ids)
        else:
            self.id = None
        self.name = name
        self.FullName = name + ":" + from_bus_pos.NodePhase
        self.status = status
        self.from_bus_pos = from_bus_pos
        self.from_bus_neg = from_bus_neg
        if primary_coil is not None:
            self.from_bus_connection_type = primary_coil.connection_type
        else:
            self.from_bus_connection_type = 'Y'  # Or some default value, if appropriate

        if secondary_coil is not None:
            self.to_bus_connection_type = secondary_coil.connection_type
        else:
            self.to_bus_connection_type = 'Y'  # Or some default value

        self.r = r
        self.x = x
        self.tr = tr
        self.ang_rad = ang * math.pi / 180.
        self.G_loss = r / (r ** 2 + x ** 2)
        self.B_loss = -x / (r ** 2 + x ** 2)
        self.G_shunt = G_shunt
        self.B_shunt = B_shunt
        self.power_rating = rating

        # Set bus connections based on transformer type
        self.set_bus_connections(to_bus_pos, to_bus_neg, primary_coil, secondary_coil)

    # Connection type checkers
    def is_wye_wye(self):
        return self.from_bus_connection_type == 'Y' and self.to_bus_connection_type == 'Y'

    def is_delta_wye(self):
        return self.from_bus_connection_type == 'D' and self.to_bus_connection_type == 'Y'

    def is_wye_delta(self):
        return self.from_bus_connection_type == 'Y' and self.to_bus_connection_type == 'D'

    def is_delta_delta(self):
        return self.from_bus_connection_type == 'D' and self.to_bus_connection_type == 'D'

    # Set bus connections based on transformer type
    def set_bus_connections(self, to_bus_pos, to_bus_neg, primary_coil, secondary_coil):
        if self.is_delta_wye():
            self.to_bus_pos = to_bus_neg
            self.to_bus_neg = secondary_coil.phase_connections[self.from_bus_neg.NodePhase]
        elif self.is_wye_wye():
            self.to_bus_pos = to_bus_pos
            self.to_bus_neg = to_bus_neg
        elif self.is_wye_delta() or self.is_delta_delta():
            # Handle Wye-Delta and Delta-Delta in the future
            pass

    # Assign variables for optimization
    def assign_xfmr_ipopt_vars(self, model):
        # General assignment of variables for different types
        if self.is_wye_wye():
            self.assign_wye_wye_vars(model)
        elif self.is_delta_wye():
            self.assign_delta_wye_vars(model)

    def assign_wye_wye_vars(self, model):
        # Wye-Wye variable assignment logic
        self.vr_A = model.ipopt_vr_list[self.from_bus_pos.NodeFullName]
        self.vi_A = model.ipopt_vi_list[self.from_bus_pos.NodeFullName]
        self.vr_a = model.ipopt_vr_list[self.to_bus_pos.NodeFullName]
        self.vi_a = model.ipopt_vi_list[self.to_bus_pos.NodeFullName]
        self.vr_aux_a = model.xfmr_vr_aux_list[self.FullName]
        self.vi_aux_a = model.xfmr_vi_aux_list[self.FullName]
        self.ir_A = model.xfmr_ir_pri_list[self.FullName]
        self.ii_A = model.xfmr_ii_pri_list[self.FullName]
        self.ir_a = model.xfmr_ir_sec_list[self.FullName]
        self.ii_a = model.xfmr_ii_sec_list[self.FullName]

    def assign_delta_wye_vars(self, model):
        # Delta-Wye variable assignment logic
        self.vr_A = model.ipopt_vr_list[self.from_bus_pos.NodeFullName]
        self.vi_A = model.ipopt_vi_list[self.from_bus_pos.NodeFullName]
        self.vr_B = model.ipopt_vr_list[self.from_bus_neg.NodeFullName]
        self.vi_B = model.ipopt_vi_list[self.from_bus_neg.NodeFullName]
        self.vr_b = model.ipopt_vr_list[self.to_bus_neg.NodeFullName]
        self.vi_b = model.ipopt_vi_list[self.to_bus_neg.NodeFullName]
        self.vr_aux_b = model.xfmr_vr_aux_list[self.FullName]
        self.vi_aux_b = model.xfmr_vi_aux_list[self.FullName]
        self.ir_AB = model.xfmr_ir_pri_list[self.FullName]
        self.ii_AB = model.xfmr_ii_pri_list[self.FullName]
        self.ir_b = model.xfmr_ir_sec_list[self.FullName]
        self.ii_b = model.xfmr_ii_sec_list[self.FullName]

    # Constraint calculation based on transformer type
    def calc_xfmr_constraints(self):
        if self.is_wye_wye():
            return self.calc_wye_wye_constraints()
        elif self.is_delta_wye():
            return self.calc_delta_wye_constraints()

    def calc_wye_wye_constraints(self):
        Ir_prim = self.ir_A
        Ii_prim = self.ii_A
        vr_prim_cons = (self.vr_A) - (self.vr_aux_a * self.tr)
        vi_prim_cons = (self.vi_A) - (self.vi_aux_a * self.tr)
        ir_aux_a_cons = self.ir_a + self.ir_A * self.tr
        ii_aux_a_cons = self.ii_a + self.ii_A * self.tr
        ir_sec_cons = self.ir_a - ((self.vr_a - self.vr_aux_a) * self.G_loss - self.B_loss * (self.vi_a - self.vi_aux_a))
        ii_sec_cons = self.ii_a - ((self.vr_a - self.vr_aux_a) * self.B_loss + self.G_loss * (self.vi_a - self.vi_aux_a))
        return Ir_prim, Ii_prim, ir_aux_a_cons, ii_aux_a_cons, vr_prim_cons, vi_prim_cons, ir_sec_cons, ii_sec_cons

    def calc_delta_wye_constraints(self):
        Ir_prim = self.ir_AB
        Ii_prim = self.ii_AB
        vr_prim_cons = (self.vr_A - self.vr_B) - ((self.vr_aux_b - self.vr_b) * self.tr)
        vi_prim_cons = (self.vi_A - self.vi_B) - ((self.vi_aux_b - self.vi_b) * self.tr)
        ir_aux_a_cons = self.ir_b + self.ir_AB * self.tr
        ii_aux_a_cons = self.ii_b + self.ii_AB * self.tr
        ir_sec_cons = self.ir_b + (self.vr_aux_b * self.G_loss - self.B_loss * self.vi_aux_b)
        ii_sec_cons = self.ii_b + (self.vr_aux_b * self.B_loss + self.G_loss * self.vi_aux_b)
        return Ir_prim, Ii_prim, ir_aux_a_cons, ii_aux_a_cons, vr_prim_cons, vi_prim_cons, ir_sec_cons, ii_sec_cons

    # Add transformer constraints to the equation lists
    def add_to_eqn_list(self,
                        KCL_equations_real,
                        KCL_equations_imag,
                        xfmr_ir_aux_constraint,
                        xfmr_ii_aux_constraint,
                        xfmr_vr_pri_constraint,
                        xfmr_vi_pri_constraint,
                        xfmr_ir_sec_constraint,
                        xfmr_ii_sec_constraint):
        if self.is_wye_wye():
            self.add_wye_wye_eqns(KCL_equations_real, KCL_equations_imag,
                                  xfmr_ir_aux_constraint, xfmr_ii_aux_constraint,
                                  xfmr_vr_pri_constraint, xfmr_vi_pri_constraint,
                                  xfmr_ir_sec_constraint, xfmr_ii_sec_constraint)
        elif self.is_delta_wye():
            self.add_delta_wye_eqns(KCL_equations_real, KCL_equations_imag,
                                    xfmr_ir_aux_constraint, xfmr_ii_aux_constraint,
                                    xfmr_vr_pri_constraint, xfmr_vi_pri_constraint,
                                    xfmr_ir_sec_constraint, xfmr_ii_sec_constraint)

    # Function for Wye-Wye transformer equations
    def add_wye_wye_eqns(self,
                         KCL_equations_real,
                         KCL_equations_imag,
                         xfmr_ir_aux_constraint,
                         xfmr_ii_aux_constraint,
                         xfmr_vr_pri_constraint,
                         xfmr_vi_pri_constraint,
                         xfmr_ir_sec_constraint,
                         xfmr_ii_sec_constraint):
        (Ir_prim, Ii_prim, ir_aux_a_cons, ii_aux_a_cons, vr_prim_cons, vi_prim_cons, ir_sec_cons, ii_sec_cons) = self.calc_wye_wye_constraints()
        self.update_constraints(KCL_equations_real, KCL_equations_imag, self.from_bus_pos.int_bus_id, Ir_prim, Ii_prim)
        self.update_constraints(KCL_equations_real, KCL_equations_imag, self.to_bus_pos.int_bus_id, self.ir_a, self.ii_a)
        self.update_constraints(xfmr_vr_pri_constraint, xfmr_vi_pri_constraint, self.from_bus_pos.int_bus_id, vr_prim_cons, vi_prim_cons)
        self.update_constraints(xfmr_ir_aux_constraint, xfmr_ii_aux_constraint, self.to_bus_pos.int_bus_id, ir_aux_a_cons, ii_aux_a_cons)
        self.update_constraints(xfmr_ir_sec_constraint, xfmr_ii_sec_constraint, self.to_bus_pos.int_bus_id, ir_sec_cons, ii_sec_cons)

    # Function for Delta-Wye transformer equations
    def add_delta_wye_eqns(self,
                           KCL_equations_real,
                           KCL_equations_imag,
                           xfmr_ir_aux_constraint,
                           xfmr_ii_aux_constraint,
                           xfmr_vr_pri_constraint,
                           xfmr_vi_pri_constraint,
                           xfmr_ir_sec_constraint,
                           xfmr_ii_sec_constraint):
        (Ir_prim, Ii_prim, ir_aux_a_cons, ii_aux_a_cons, vr_prim_cons, vi_prim_cons, ir_sec_cons, ii_sec_cons) = self.calc_delta_wye_constraints()
        self.update_constraints(KCL_equations_real, KCL_equations_imag, self.from_bus_pos.int_bus_id, Ir_prim, Ii_prim)
        self.update_constraints(KCL_equations_real, KCL_equations_imag, self.from_bus_neg.int_bus_id, -Ir_prim, -Ii_prim)
        self.update_constraints(KCL_equations_real, KCL_equations_imag, self.to_bus_neg.int_bus_id, -self.ir_b, -self.ii_b)
        self.update_constraints(xfmr_vr_pri_constraint, xfmr_vi_pri_constraint, self.from_bus_pos.int_bus_id, vr_prim_cons, vi_prim_cons)
        self.update_constraints(xfmr_ir_aux_constraint, xfmr_ii_aux_constraint, self.to_bus_neg.int_bus_id, ir_aux_a_cons, ii_aux_a_cons)
        self.update_constraints(xfmr_ir_sec_constraint, xfmr_ii_sec_constraint, self.to_bus_neg.int_bus_id, ir_sec_cons, ii_sec_cons)

    # Update KCL equations for real and imaginary components
    def update_constraints(self, eqns_real, eqns_imag, bus_id, ir, ii):
        if bus_id in eqns_real:
            eqns_real[bus_id] += ir
        else:
            eqns_real[bus_id] = ir

        if bus_id in eqns_imag:
            eqns_imag[bus_id] += ii
        else:
            eqns_imag[bus_id] = ii
