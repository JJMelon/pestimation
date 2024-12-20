from typing import List
from itertools import count
import numpy as np
from sympy import symbols
from models.wellknownvariables import tx_factor
from pyomo.environ import Var as PyomoVar, value
from typing import Union

class CenterTapTransformerCoil:
    def __init__(self, nominal_voltage, rated_power, connection_type, voltage_limit, resistance=None, reactance=None, from_node=None, to_node=None, primary_node=None, sending_node=None):
        self.nominal_voltage = nominal_voltage
        self.rated_power = rated_power
        self.connection_type = connection_type
        self.voltage_limit = voltage_limit
        self.resistance = resistance
        self.reactance = reactance
        self.from_node = from_node  # for primary node
        self.to_node = to_node  # general usage for to_node
        self.primary_node = primary_node  # specific for primary coil
        self.sending_node = sending_node  # specific for sending nodes in other coils


class CenterTapTransformer():
    _ids = count(0)

    def __init__(self,
                 name,
                 coil_1,
                 coil_2,
                 coil_3,
                 phase,
                 turn_ratio,
                 power_rating,
                 g_shunt,
                 b_shunt
                 ):
        self.id = self._ids.__next__()
        self.name = name
        self.coils: List[CenterTapTransformerCoil]
        self.coils = [coil_1, coil_2, coil_3]
        self.phases = phase
        self.FullName = name + ":" + phase
        self.turn_ratio = turn_ratio
        self.power_rating = power_rating
        self.g_shunt = g_shunt
        self.b_shunt = b_shunt

        # Values for the primary coil impedance stamps, converted out of per-unit
        r0 = self.coils[0].resistance * \
            self.coils[0].nominal_voltage ** 2 / self.power_rating
        x0 = self.coils[0].reactance * \
            self.coils[0].nominal_voltage ** 2 / self.power_rating
        self.g0 = r0 / (r0**2 + x0**2) if not (r0 == 0 and x0 == 0) else 0
        self.b0 = -x0 / (r0**2 + x0**2) if not (r0 == 0 and x0 == 0) else 0

        # Values for the first triplex coil impedance stamps, converted out of per-unit
        r1 = self.coils[1].resistance * \
            self.coils[1].nominal_voltage ** 2 / self.power_rating
        x1 = self.coils[1].reactance * \
            self.coils[1].nominal_voltage ** 2 / self.power_rating
        self.g1 = r1 / (r1**2 + x1**2) if not (r1 == 0 and x1 == 0) else 0
        self.b1 = -x1 / (r1**2 + x1**2) if not (r1 == 0 and x1 == 0) else 0

        # Values for the second triplex coil impedance stamps, converted out of per-unit
        r2 = self.coils[2].resistance * \
            self.coils[2].nominal_voltage ** 2 / self.power_rating
        x2 = self.coils[2].reactance * \
            self.coils[2].nominal_voltage ** 2 / self.power_rating
        self.g2 = r2 / (r2**2 + x2**2) if not (r2 == 0 and x2 == 0) else 0
        self.b2 = -x2 / (r2**2 + x2**2) if not (r2 == 0 and x2 == 0) else 0

    def assign_nodes(self, node_index, optimization_enabled):
        from_bus = self.coils[0].primary_node
        L1_bus = self.coils[1].sending_node
        L2_bus = self.coils[2].sending_node

        self.node_L1_Ir = next(node_index)
        self.node_L1_Ii = next(node_index)
        self.node_L2_Ir = next(node_index)
        self.node_L2_Ii = next(node_index)

        index_map = {}
        index_map[Vr_pri] = from_bus.node_Vr
        index_map[Vi_pri] = from_bus.node_Vi
        index_map[Ir_L1] = self.node_L1_Ir
        index_map[Ii_L1] = self.node_L1_Ii
        index_map[Ir_L2] = self.node_L2_Ir
        index_map[Ii_L2] = self.node_L2_Ii
        index_map[Vr_L1] = L1_bus.node_Vr
        index_map[Vi_L1] = L1_bus.node_Vi
        index_map[Vr_L2] = L2_bus.node_Vr
        index_map[Vi_L2] = L2_bus.node_Vi

        if optimization_enabled:
            index_map[Lr_pri] = from_bus.node_lambda_Vr
            index_map[Li_pri] = from_bus.node_lambda_Vi
            index_map[Lir_L1] = next(node_index)
            index_map[Lii_L1] = next(node_index)
            index_map[Lir_L2] = next(node_index)
            index_map[Lii_L2] = next(node_index)
            index_map[Lr_L1] = L1_bus.node_lambda_Vr
            index_map[Li_L1] = L1_bus.node_lambda_Vi
            index_map[Lr_L2] = L2_bus.node_lambda_Vr
            index_map[Li_L2] = L2_bus.node_lambda_Vi
        else:
            index_map[Lr_pri] = SKIP
            index_map[Li_pri] = SKIP
            index_map[Lir_L1] = SKIP
            index_map[Lii_L1] = SKIP
            index_map[Lir_L2] = SKIP
            index_map[Lii_L2] = SKIP
            index_map[Lr_L1] = SKIP
            index_map[Li_L1] = SKIP
            index_map[Lr_L2] = SKIP
            index_map[Li_L2] = SKIP


    def get_connections(self):
        return [
            (self.coils[0].from_node, self.coils[1].to_node),
            (self.coils[0].from_node, self.coils[2].to_node)
        ]

    def assign_CT_ipopt_vars(self, model):

        # Primary
        self.ipopt_vr_pri_CT: Union[PyomoVar, float]
        self.ipopt_vi_pri_CT: Union[PyomoVar, float]
        self.ipopt_vr_pri_aux: Union[PyomoVar, float]
        self.ipopt_vi_pri_aux: Union[PyomoVar, float]

        # Secondary
        self.ipopt_vr_sec1: Union[PyomoVar, float]
        self.ipopt_vi_sec1: Union[PyomoVar, float]
        self.ipopt_vr_sec2: Union[PyomoVar, float]
        self.ipopt_vi_sec2: Union[PyomoVar, float]
        self.ipopt_vr_aux_sec1: Union[PyomoVar, float]
        self.ipopt_vi_aux_sec1: Union[PyomoVar, float]
        self.ipopt_vr_aux_sec2: Union[PyomoVar, float]
        self.ipopt_vi_aux_sec2: Union[PyomoVar, float]

        #current
        self.ipopt_ir_from:Union[PyomoVar, float]
        self.ipopt_ii_from:Union[PyomoVar, float]
        self.ipopt_ir_to1: Union[PyomoVar, float]
        self.ipopt_ii_to1: Union[PyomoVar, float]
        self.ipopt_ir_to2: Union[PyomoVar, float]
        self.ipopt_ii_to2: Union[PyomoVar, float]

        # Primary
        self.ipopt_vr_pri_CT = model.ipopt_vr_list[self.coils[0].from_node.NodeFullName]
        self.ipopt_vi_pri_CT = model.ipopt_vi_list[self.coils[0].from_node.NodeFullName]
        self.ipopt_vr_pri_aux = model.ipopt_vr_list[self.coils[0].primary_node.NodeFullName]
        self.ipopt_vi_pri_aux = model.ipopt_vi_list[self.coils[0].primary_node.NodeFullName]
        self.ipopt_ir_from = model.CT_ir_pri_list[self.FullName]
        self.ipopt_ii_from = model.CT_ii_pri_list[self.FullName]

        # Secondary, coil 1
        self.ipopt_vr_sec1 = model.ipopt_vr_list[self.coils[1].to_node.NodeFullName]
        self.ipopt_vi_sec1 = model.ipopt_vi_list[self.coils[1].to_node.NodeFullName]
        self.ipopt_vr_aux_sec1 = model.ipopt_vr_list[self.coils[1].sending_node.NodeFullName]
        self.ipopt_vi_aux_sec1 = model.ipopt_vi_list[self.coils[1].sending_node.NodeFullName]
        self.ipopt_ir_to1 = model.CT_ir_sec1_list[self.FullName]
        self.ipopt_ii_to1 = model.CT_ii_sec1_list[self.FullName]

        # Secondary, coil 2
        self.ipopt_vr_sec2 = model.ipopt_vr_list[self.coils[2].to_node.NodeFullName]
        self.ipopt_vi_sec2 = model.ipopt_vi_list[self.coils[2].to_node.NodeFullName]
        self.ipopt_vr_aux_sec2 = model.ipopt_vr_list[self.coils[2].sending_node.NodeFullName]
        self.ipopt_vi_aux_sec2 = model.ipopt_vi_list[self.coils[2].sending_node.NodeFullName]
        self.ipopt_ir_to2 = model.CT_ir_sec2_list[self.FullName]
        self.ipopt_ii_to2 = model.CT_ii_sec2_list[self.FullName]

    def calc_CT_data(self):

        # Current at primary from node
        Ir_pri_from = self.ipopt_ir_from
        Ii_pri_from = self.ipopt_ii_from

        # Current at secondary nodes
        Ir_sec_to1 = self.ipopt_ir_to1
        Ii_sec_to1 = self.ipopt_ii_to1

        Ir_sec_to2 = self.ipopt_ir_to2
        Ii_sec_to2 = self.ipopt_ii_to2

        # Excitation current at primary
        Iex_r = self.g_shunt * self.ipopt_vr_pri_CT - self.b_shunt * self.ipopt_vi_pri_CT
        Iex_i = self.g_shunt * self.ipopt_vi_pri_CT + self.b_shunt * self.ipopt_vr_pri_CT

        # Total primary current components
        I1_r = Ir_pri_from + Iex_r
        I1_i = Ii_pri_from + Iex_i

        # Current at primary
        Ir_aux_0 = (self.ipopt_vr_pri_CT-self.ipopt_vr_pri_aux)*self.g0 - \
            (self.ipopt_vi_pri_CT-self.ipopt_vi_pri_aux)*self.b0
        Ii_aux_0 = (self.ipopt_vr_pri_CT-self.ipopt_vr_pri_aux)*self.b0 + \
            (self.ipopt_vi_pri_CT-self.ipopt_vi_pri_aux)*self.g0

        # Current at secondary node 1
        Ir_aux_1 = (self.ipopt_vr_sec1 - self.ipopt_vr_aux_sec1) * self.g1 - \
            (self.ipopt_vi_sec1 - self.ipopt_vi_aux_sec1) * self.b1
        Ii_aux_1 = (self.ipopt_vr_sec1 - self.ipopt_vr_aux_sec1) * self.b1 + \
            (self.ipopt_vi_sec1 - self.ipopt_vi_aux_sec1) * self.g1

        # Current at secondary node 2
        Ir_aux_2 = (self.ipopt_vr_sec2 - self.ipopt_vr_aux_sec2) * self.g2 - \
            (self.ipopt_vi_sec2 - self.ipopt_vi_aux_sec2) * self.b2
        Ii_aux_2 = (self.ipopt_vr_sec2 - self.ipopt_vr_aux_sec2) * self.b2 + \
            (self.ipopt_vi_sec2 - self.ipopt_vi_aux_sec2) * self.g2

        # Constraint build up
        Vr_sec_cons1 = self.ipopt_vr_aux_sec1*self.turn_ratio - self.ipopt_vr_pri_aux
        Vi_sec_cons1 = self.ipopt_vi_aux_sec1*self.turn_ratio - self.ipopt_vi_pri_aux
        Vr_sec_cons2 = self.ipopt_vr_aux_sec2*self.turn_ratio + self.ipopt_vr_pri_aux
        Vi_sec_cons2 = self.ipopt_vi_aux_sec2*self.turn_ratio + self.ipopt_vi_pri_aux

        Ir_pri_cons = self.ipopt_ir_from + (1/self.turn_ratio) * \
            self.ipopt_ir_to1 - (1/self.turn_ratio)*self.ipopt_ir_to2
        Ii_pri_cons = self.ipopt_ii_from + (1/self.turn_ratio) * \
            self.ipopt_ii_to1 - (1/self.turn_ratio)*self.ipopt_ii_to2

        return (I1_r, I1_i, Ir_sec_to1, Ii_sec_to1, Ir_sec_to2, Ii_sec_to2,
                Ir_aux_0, Ii_aux_0, Ir_aux_1, Ii_aux_1, Ir_aux_2, Ii_aux_2,
                Vr_sec_cons1, Vi_sec_cons1, Vr_sec_cons2, Vi_sec_cons2,
                Ir_pri_cons, Ii_pri_cons)

    def add_to_eqn_list_CT(self,
                           KCL_equations_real, KCL_equations_imag,
                           Vr_sec_cons1_dict, Vi_sec_cons1_dict,
                           Vr_sec_cons2_dict, Vi_sec_cons2_dict,
                           Ir_pri_cons_dict, Ii_pri_cons_dict):
        #
        # Extract values from calc_CT_data
        (Ir_pri_from,Ii_pri_from, Ir_sec_to1, Ii_sec_to1,
         Ir_sec_to2, Ii_sec_to2,Ir_aux_0, Ii_aux_0, Ir_aux_1, Ii_aux_1, Ir_aux_2, Ii_aux_2,
         vr_sec_cons1, vi_sec_cons1, vr_sec_cons2, vi_sec_cons2,
         ir_pri_cons, ii_pri_cons) = self.calc_CT_data()

        # Updating KCL equations for primary and secondary sides
        self.update_kcl(KCL_equations_real, KCL_equations_imag,
                        self.coils[0].from_node.int_bus_id, Ir_pri_from, Ii_pri_from)
        self.update_kcl(KCL_equations_real, KCL_equations_imag,
                        self.coils[1].to_node.int_bus_id, Ir_sec_to1, Ii_sec_to1)
        self.update_kcl(KCL_equations_real, KCL_equations_imag,
                        self.coils[2].to_node.int_bus_id, -Ir_sec_to2, -Ii_sec_to2)

        # Updating KCL equations for primary and secondary sides (aux)
        self.update_kcl(KCL_equations_real, KCL_equations_imag,
                        self.coils[0].primary_node.int_bus_id, -Ir_aux_0 +
                        self.ipopt_ir_from, -Ii_aux_0 + self.ipopt_ii_from)
        self.update_kcl(KCL_equations_real, KCL_equations_imag,
                        self.coils[1].sending_node.int_bus_id, -Ir_aux_1 +
                        self.ipopt_ir_to1,  -Ii_aux_1 + self.ipopt_ii_to1)
        self.update_kcl(KCL_equations_real, KCL_equations_imag,
                        self.coils[2].sending_node.int_bus_id, Ir_aux_2 -
                        self.ipopt_ir_to2,  Ii_aux_2 - self.ipopt_ii_to2)

        # Updating voltage and current constraints in the provided dictionaries
        Vr_sec_cons1_dict[self.id] = vr_sec_cons1
        Vi_sec_cons1_dict[self.id] = vi_sec_cons1
        Vr_sec_cons2_dict[self.id] = vr_sec_cons2
        Vi_sec_cons2_dict[self.id] = vi_sec_cons2
        Ir_pri_cons_dict[self.id] = ir_pri_cons
        Ii_pri_cons_dict[self.id] = ii_pri_cons

    def print_transformer_current(self):
        pass

    def update_kcl(self,
                   KCL_equations_real,
                   KCL_equations_imag,
                   int_bus_id,
                   Ir,
                   Ii):
        # Updating KCL equations for a given bus
        if int_bus_id in KCL_equations_real:
            KCL_equations_real[int_bus_id] += Ir
        else:
            KCL_equations_real[int_bus_id] = Ir
        #
        if int_bus_id in KCL_equations_imag:
            KCL_equations_imag[int_bus_id] += Ii
        else:
            KCL_equations_imag[int_bus_id] = Ii
