import typing
import numpy as np
from models.components.bus import GROUND
from models.components.bus import Bus
from logic.network.networkmodel import DxNetworkModel
from models.wellknownvariables import Vr_from, Vr_to, Vi_from, Vi_to, Lr_from, Lr_to, Li_from, Li_to
from pyomo.environ import Var, ConcreteModel, ConstraintList, Objective
from typing import Union, Optional
from pyomo.environ import Var as PyomoVar
import os

# output_file = "shunt_admittances_output.txt"
# def write_shunt_admittances_to_file(shunt_admittances, file_path):
#     with open(file_path, "a") as file:
#         file.write(str(shunt_admittances) + "\n")

# output_file2 = "admittances_output.txt"
# def write_admittances_to_file(admittances, file_path):
#     with open(file_path, "a") as file:
#         file.write(str(admittances) + "\n")

def calcInverse(Zmatrix):
    num_nnz = np.count_nonzero(Zmatrix)
    if num_nnz == 1:
        # Division in the next step will result in 0 + 0j in Y
        Zmatrix_red = Zmatrix[Zmatrix != 0 + 0j]
        #Zmatrix[Zmatrix == 0 + 0j] = np.inf
        #Zmatrix_red = Zmatrix
        #Ymatrix = np.divide(np.ones(np.shape(Zmatrix), dtype=complex), Zmatrix)
        Ymatrix_red = (np.divide(np.ones(np.shape(Zmatrix_red), dtype=complex), Zmatrix_red)).reshape(1, 1)
        return Ymatrix_red
    # Convert Matrix to Ymatrix
    # Find the rows of zero
    _rowZeroIdx = np.all(Zmatrix == 0, axis=1)
    Zmatrix_red = Zmatrix[~_rowZeroIdx]
    # Find the cols of zero
    _colZeroIdx = np.all(Zmatrix_red == 0, axis=0)
    Zmatrix_red = Zmatrix_red[:, ~_colZeroIdx]
    # Find the inverse of reduced matrix
    Ymatrix_red = np.linalg.inv(Zmatrix_red)
    # Remap to original format
    if len(Ymatrix_red) == 2:
        _rowIdx = list(_rowZeroIdx).index(True)
        _colIdx = list(_colZeroIdx).index(True)
        Ymatrix = np.insert(Ymatrix_red, _rowIdx, 0 + 1j * 0, axis=0)
        Ymatrix = np.insert(Ymatrix, _colIdx, 0 + 1j * 0, axis=1)
    else:
        Ymatrix = Ymatrix_red
    return Ymatrix_red

def removeZeros(Y):
    # Find the rows of zero
    _rowZeroIdx = np.all(Y == 0, axis=1)
    Y_red = Y[~_rowZeroIdx]
    # Find the cols of zero
    _colZeroIdx = np.all(Y_red == 0, axis=0)
    Y_red = Y_red[:, ~_colZeroIdx]
    return Y_red


class UnbalancedLinePhase():
    def __init__(self,
                 from_element,
                 to_element,
                 phase,
                 phases,
                 admittances,
                 shunt_admittances,
                 network_model
                 ):
        self.from_element = from_element
        self.to_element = to_element
        self.phase = phase
        # Necessary for mutual
        self.network_model = network_model
        self.get_nodes(self.network_model)
        self.phases = phases
        (from_bus, to_bus) = self.get_nodes(network_model)
        self.from_bus_idx = from_bus.int_bus_id
        self.to_bus_idx = to_bus.int_bus_id
        #self.from_bus_idx = Bus.node2index_dict[(self.from_element, self.phase)]
        #### trying a solution ####
        #self.to_bus_idx = Bus.node2index_dict[(self.to_element, self.phase)]

        # Split into conductances and susceptances
        # make into a python list so we can change entries to pyomo variables in estimation
        self.susceptances = admittances.imag.tolist()
        self.conductances = admittances.real.tolist()
        self.shunt_admittances = shunt_admittances
        if "A" in self.phases or "B" in self.phases or "C" in self.phases:
            self.is_triplex = False
        elif "1" in self.phases or "2" in self.phases:
            self.is_triplex = True
        # write_shunt_admittances_to_file(self.shunt_admittances, output_file)
        # write_admittances_to_file(admittances, output_file2)


    def get_nodes(self, state: DxNetworkModel):
        from_bus = state.bus_name_map[self.from_element + "_" + self.phase]
        to_bus = state.bus_name_map[self.to_element + "_" + self.phase]
        return from_bus, to_bus

    def get_nodes_phase(self, state: DxNetworkModel, node_name, phase):
        bus_id = state.bus_name_map[node_name + "_" + phase].int_bus_id
        return bus_id

    # ----------------------------------------------------------------------------------------------------------- #
    def create_ipopt_vars(self, model, bus):
        # We need to get PyomoVars for self and mutual impedance in here
        # print(self.phase, self.phases)
        # Loading the bus id for phases A, B, C (both real and imag)

        self.ipopt_vr_from: Union[PyomoVar, np.array]
        self.ipopt_vi_from: Union[PyomoVar, np.array]
        self.ipopt_vr_to: Union[PyomoVar, np.array]
        self.ipopt_vi_to: Union[PyomoVar, np.array]

        # Checking and assigning IPOPT variables only for existing phases
        if "A" in self.phases:
            phaseA_from_id = self.get_nodes_phase(self.network_model, self.from_element, "A")
            phaseA_to_id = self.get_nodes_phase(self.network_model, self.to_element, "A")
            self.ip_vr_fromA = bus[phaseA_from_id].ipopt_vr
            self.ip_vi_fromA = bus[phaseA_from_id].ipopt_vi
            self.ip_vr_toA = bus[phaseA_to_id].ipopt_vr
            self.ip_vi_toA = bus[phaseA_to_id].ipopt_vi

        if "B" in self.phases:
            phaseB_from_id = self.get_nodes_phase(self.network_model, self.from_element, "B")
            phaseB_to_id = self.get_nodes_phase(self.network_model, self.to_element, "B")
            self.ip_vr_fromB = bus[phaseB_from_id].ipopt_vr
            self.ip_vi_fromB = bus[phaseB_from_id].ipopt_vi
            self.ip_vr_toB = bus[phaseB_to_id].ipopt_vr
            self.ip_vi_toB = bus[phaseB_to_id].ipopt_vi

        if "C" in self.phases:
            phaseC_from_id = self.get_nodes_phase(self.network_model, self.from_element, "C")
            phaseC_to_id = self.get_nodes_phase(self.network_model, self.to_element, "C")
            self.ip_vr_fromC = bus[phaseC_from_id].ipopt_vr
            self.ip_vi_fromC = bus[phaseC_from_id].ipopt_vi
            self.ip_vr_toC = bus[phaseC_to_id].ipopt_vr
            self.ip_vi_toC = bus[phaseC_to_id].ipopt_vi

    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag):
        if not self.is_triplex:
            (Ir_from, Ir_to) = self.calc_real_current()
            (Ii_from, Ii_to) = self.calc_imag_current()
            Ir_sh_from, Ii_sh_from, Ir_sh_to, Ii_sh_to = self.calc_shunt_current()

            if self.from_bus_idx in KCL_equations_real:
                KCL_equations_real[self.from_bus_idx] += Ir_from + Ir_sh_from
            else:
                KCL_equations_real[self.from_bus_idx] = Ir_from + Ir_sh_from

            if self.to_bus_idx in KCL_equations_real:
                KCL_equations_real[self.to_bus_idx] += Ir_to + Ir_sh_to
            else:
                KCL_equations_real[self.to_bus_idx] = Ir_to + Ir_sh_to

            if self.from_bus_idx in KCL_equations_imag:
                KCL_equations_imag[self.from_bus_idx] += Ii_from + Ii_sh_from
            else:
                KCL_equations_imag[self.from_bus_idx] = Ii_from + Ii_sh_from

            if self.to_bus_idx in KCL_equations_imag:
                KCL_equations_imag[self.to_bus_idx] += Ii_to + Ii_sh_to
            else:
                KCL_equations_imag[self.to_bus_idx] = Ii_to + Ii_sh_to

        elif self.is_triplex:
            (Ir_from1, Ir_to1, Ir_from2, Ir_to2) = self.calc_real_current_triplex()
            (Ii_from1, Ii_to1, Ii_from2, Ii_to2) = self.calc_imag_current_triplex()

            if self.phase == "1":
                Ir_from = Ir_from1
                Ir_to = Ir_to1
                Ii_from = Ii_from1
                Ii_to = Ii_to1
            elif self.phase == "2":
                Ir_from = Ir_from2
                Ir_to = Ir_to2
                Ii_from = Ii_from2
                Ii_to = Ii_to2

            if self.from_bus_idx in KCL_equations_real:
                KCL_equations_real[self.from_bus_idx] += Ir_from
            else:
                KCL_equations_real[self.from_bus_idx] = Ir_from
            if self.to_bus_idx in KCL_equations_real:
                KCL_equations_real[self.to_bus_idx] += Ir_to
            else:
                KCL_equations_real[self.to_bus_idx] = Ir_to
            if self.from_bus_idx in KCL_equations_imag:
                KCL_equations_imag[self.from_bus_idx] += Ii_from
            else:
                KCL_equations_imag[self.from_bus_idx] = Ii_from
            if self.to_bus_idx in KCL_equations_imag:
                KCL_equations_imag[self.to_bus_idx] += Ii_to
            else:
                KCL_equations_imag[self.to_bus_idx] = Ii_to


    def calc_real_current(self):
        # Mapping of phase to indices in the conductances and susceptances matrices
        phase_indices = {"A": 0, "B": 1, "C": 2}
        mutual_phases = {
            "A": ["B", "C"],
            "B": ["A", "C"],
            "C": ["A", "B"]
        }

        # Determine index for the current phase
        phase_idx = phase_indices.get(self.phase)
        if phase_idx is None:
            raise ValueError(f"Invalid phase: {self.phase}")

        # Get indices for mutual phases
        mutual_phase_indices = [phase_indices[p] for p in mutual_phases[self.phase] if p in self.phases]

        # Self-admittances
        self.Gself = self.conductances[phase_idx][phase_idx]
        self.Bself = self.susceptances[phase_idx][phase_idx]

        # Calculate the contribution of the self-admittance
        Ir_from = ((getattr(self, f"ip_vr_from{self.phase}") - getattr(self, f"ip_vr_to{self.phase}")) * self.Gself -
                   (getattr(self, f"ip_vi_from{self.phase}") - getattr(self, f"ip_vi_to{self.phase}")) * self.Bself)

        # Add contributions from mutual admittances
        for i, mutual_idx in enumerate(mutual_phase_indices):
            mutual_phase = mutual_phases[self.phase][i]
            Gmutual = self.conductances[phase_idx][mutual_idx]
            Bmutual = self.susceptances[phase_idx][mutual_idx]

            Ir_from += ((getattr(self, f"ip_vr_from{mutual_phase}") - getattr(self, f"ip_vr_to{mutual_phase}")) * Gmutual -
                        (getattr(self, f"ip_vi_from{mutual_phase}") - getattr(self, f"ip_vi_to{mutual_phase}")) * Bmutual)

        # Opposite current direction
        Ir_to = -Ir_from
        return Ir_from, Ir_to

    def calc_imag_current(self):
        # Phase to index mapping
        phase_indices = {"A": 0, "B": 1, "C": 2}

        # Determine index for the current phase
        phase_idx = phase_indices.get(self.phase)
        if phase_idx is None:
            raise ValueError(f"Invalid phase: {self.phase}")

        # Extract self-conductance and self-susceptance
        self.Gself = self.conductances[phase_idx][phase_idx]
        self.Bself = self.susceptances[phase_idx][phase_idx]

        # Initialize current from self-conductance and self-susceptance
        Ii_from = (
            (getattr(self, f"ip_vi_from{self.phase}") - getattr(self, f"ip_vi_to{self.phase}")) * self.Gself +
            (getattr(self, f"ip_vr_from{self.phase}") - getattr(self, f"ip_vr_to{self.phase}")) * self.Bself
        )

        # Handle mutual phases dynamically based on available phases
        for mutual_phase, mutual_idx in phase_indices.items():
            if mutual_phase != self.phase and mutual_phase in self.phases:
                Gmutual = self.conductances[phase_idx][mutual_idx]
                Bmutual = self.susceptances[phase_idx][mutual_idx]
                Ii_from += (
                    (getattr(self, f"ip_vi_from{mutual_phase}") - getattr(self, f"ip_vi_to{mutual_phase}")) * Gmutual +
                    (getattr(self, f"ip_vr_from{mutual_phase}") - getattr(self, f"ip_vr_to{mutual_phase}")) * Bmutual
                )

        # Opposite direction current
        Ii_to = -Ii_from
        return Ii_from, Ii_to

    def calc_shunt_current(self):
        Ir_sh_from = 0
        Ii_sh_from = 0
        Ir_sh_to = 0
        Ii_sh_to = 0

        if self.shunt_admittances is not None:
            if "A" in self.phases and "B" in self.phases and "C" in self.phases:
                # Case when all three phases are present
                if self.phase == "A":
                    B_shunt = self.shunt_admittances[0][0].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromA / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromA / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toA / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toA / 2
                    if hasattr(self, 'ip_vr_fromB'):
                        B_shunt_mutual = self.shunt_admittances[0][1].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromB / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromB / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toB / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toB / 2
                    if hasattr(self, 'ip_vr_fromC'):
                        B_shunt_mutual = self.shunt_admittances[0][2].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromC / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromC / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toC / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toC / 2
                elif self.phase == "B":
                    B_shunt = self.shunt_admittances[1][1].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromB / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromB / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toB / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toB / 2
                    if hasattr(self, 'ip_vr_fromA'):
                        B_shunt_mutual = self.shunt_admittances[1][0].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromA / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromA / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toA / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toA / 2
                    if hasattr(self, 'ip_vr_fromC'):
                        B_shunt_mutual = self.shunt_admittances[1][2].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromC / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromC / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toC / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toC / 2
                elif self.phase == "C":
                    B_shunt = self.shunt_admittances[2][2].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromC / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromC / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toC / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toC / 2
                    if hasattr(self, 'ip_vr_fromA'):
                        B_shunt_mutual = self.shunt_admittances[2][0].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromA / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromA / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toA / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toA / 2
                    if hasattr(self, 'ip_vr_fromB'):
                        B_shunt_mutual = self.shunt_admittances[2][1].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromB / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromB / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toB / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toB / 2

            elif "A" in self.phases and "B" in self.phases and "C" not in self.phases:
                # Case when only phases A and B are present
                if self.phase == "A":
                    B_shunt = self.shunt_admittances[0][0].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromA / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromA / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toA / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toA / 2
                    if hasattr(self, 'ip_vr_fromB'):
                        B_shunt_mutual = self.shunt_admittances[0][1].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromB / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromB / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toB / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toB / 2
                elif self.phase == "B":
                    B_shunt = self.shunt_admittances[1][1].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromB / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromB / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toB / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toB / 2
                    if hasattr(self, 'ip_vr_fromA'):
                        B_shunt_mutual = self.shunt_admittances[1][0].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromA / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromA / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toA / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toA / 2

            elif "B" in self.phases and "C" in self.phases and "A" not in self.phases:
                # Case when only phases B and C are present
                if self.phase == "B":
                    B_shunt = self.shunt_admittances[0][0].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromB / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromB / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toB / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toB / 2
                    if hasattr(self, 'ip_vr_fromC'):
                        B_shunt_mutual = self.shunt_admittances[0][1].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromC / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromC / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toC / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toC / 2
                elif self.phase == "C":
                    B_shunt = self.shunt_admittances[1][1].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromC / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromC / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toC / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toC / 2
                    if hasattr(self, 'ip_vr_fromB'):
                        B_shunt_mutual = self.shunt_admittances[1][0].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromB / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromB / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toB / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toB / 2

            elif "A" in self.phases and "C" in self.phases and "B" not in self.phases:
                # Case when only phases A and C are present
                if self.phase == "A":
                    B_shunt = self.shunt_admittances[0][0].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromA / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromA / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toA / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toA / 2
                    if hasattr(self, 'ip_vr_fromC'):
                        B_shunt_mutual = self.shunt_admittances[0][1].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromC / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromC / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toC / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toC / 2
                elif self.phase == "C":
                    B_shunt = self.shunt_admittances[1][1].imag
                    Ir_sh_from = -B_shunt * self.ip_vi_fromC / 2
                    Ii_sh_from = B_shunt * self.ip_vr_fromC / 2
                    Ir_sh_to = -B_shunt * self.ip_vi_toC / 2
                    Ii_sh_to = B_shunt * self.ip_vr_toC / 2
                    if hasattr(self, 'ip_vr_fromA'):
                        B_shunt_mutual = self.shunt_admittances[1][0].imag
                        Ir_sh_from += -B_shunt_mutual * self.ip_vi_fromA / 2
                        Ii_sh_from += B_shunt_mutual * self.ip_vr_fromA / 2
                        Ir_sh_to += -B_shunt_mutual * self.ip_vi_toA / 2
                        Ii_sh_to += B_shunt_mutual * self.ip_vr_toA / 2

            elif "A" in self.phases and "B" not in self.phases and "C" not in self.phases:
                # Case when only phase A is present
                B_shunt = self.shunt_admittances[0][0].imag
                Ir_sh_from = -B_shunt * self.ip_vi_fromA / 2
                Ii_sh_from = B_shunt * self.ip_vr_fromA / 2
                Ir_sh_to = -B_shunt * self.ip_vi_toA / 2
                Ii_sh_to = B_shunt * self.ip_vr_toA / 2

            elif "B" in self.phases and "A" not in self.phases and "C" not in self.phases:
                # Case when only phase B is present
                B_shunt = self.shunt_admittances[0][0].imag
                Ir_sh_from = -B_shunt * self.ip_vi_fromB / 2
                Ii_sh_from = B_shunt * self.ip_vr_fromB / 2
                Ir_sh_to = -B_shunt * self.ip_vi_toB / 2
                Ii_sh_to = B_shunt * self.ip_vr_toB / 2

            elif "C" in self.phases and "A" not in self.phases and "B" not in self.phases:
                # Case when only phase C is present
                B_shunt = self.shunt_admittances[0][0].imag
                Ir_sh_from = -B_shunt * self.ip_vi_fromC / 2
                Ii_sh_from = B_shunt * self.ip_vr_fromC / 2
                Ir_sh_to = -B_shunt * self.ip_vi_toC / 2
                Ii_sh_to = B_shunt * self.ip_vr_toC / 2

        return Ir_sh_from, Ii_sh_from, Ir_sh_to, Ii_sh_to



    # ----------------------------------------------------------------------------------------------------------- #
    def create_ipopt_vars_triplex(self, model, bus):
        # Initializing IPOPT variables for from and to nodes of the triplex line
        self.ipopt_vr_from1: Union[PyomoVar, np.array]
        self.ipopt_vi_from1: Union[PyomoVar, np.array]
        self.ipopt_vr_from2: Union[PyomoVar, np.array]
        self.ipopt_vi_from2: Union[PyomoVar, np.array]
        self.ipopt_vr_to1: Union[PyomoVar, np.array]
        self.ipopt_vi_to1: Union[PyomoVar, np.array]
        self.ipopt_vr_to2: Union[PyomoVar, np.array]
        self.ipopt_vi_to2: Union[PyomoVar, np.array]

        # Triplex line uses "1" and "2" for main conductors
        phase1_from_id = self.get_nodes_phase(self.network_model, self.from_element, "1")
        phase2_from_id = self.get_nodes_phase(self.network_model, self.from_element, "2")

        phase1_to_id = self.get_nodes_phase(self.network_model, self.to_element, "1")
        phase2_to_id = self.get_nodes_phase(self.network_model, self.to_element, "2")

        # Assigning IPOPT variables for voltage real and imaginary parts for "from" nodes
        self.ip_vr_from1 = bus[phase1_from_id].ipopt_vr
        self.ip_vi_from1 = bus[phase1_from_id].ipopt_vi
        self.ip_vr_from2 = bus[phase2_from_id].ipopt_vr
        self.ip_vi_from2 = bus[phase2_from_id].ipopt_vi

        # Assigning IPOPT variables for voltage real and imaginary parts for "to" nodes
        self.ip_vr_to1 = bus[phase1_to_id].ipopt_vr
        self.ip_vi_to1 = bus[phase1_to_id].ipopt_vi
        self.ip_vr_to2 = bus[phase2_to_id].ipopt_vr
        self.ip_vi_to2 = bus[phase2_to_id].ipopt_vi

    def calc_real_current_triplex(self):
        # Self admittance for conductors 1 and 2
        self.Gself1 = self.admittances[0][0].real
        self.Bself1 = self.admittances[0][0].imag
        self.Gself2 = self.admittances[1][1].real
        self.Bself2 = self.admittances[1][1].imag

        # Mutual admittance between conductors 1 and 2
        self.Gmutual1 = self.admittances[0][1].real
        self.Bmutual1 = self.admittances[0][1].imag
        self.Gmutual2 = self.admittances[1][0].real
        self.Bmutual2 = self.admittances[1][0].imag

        # Real current calculation for each conductor
        Ir_from1 = ((self.ip_vr_from1 - self.ip_vr_to1) * self.Gself1 -
                    (self.ip_vi_from1 - self.ip_vi_to1) * self.Bself1 +
                    (self.ip_vr_from2 - self.ip_vr_to2) * self.Gmutual1 -
                    (self.ip_vi_from2 - self.ip_vi_to2) * self.Bmutual1)

        Ir_from2 = ((self.ip_vr_from2 - self.ip_vr_to2) * self.Gself2 -
                    (self.ip_vi_from2 - self.ip_vi_to2) * self.Bself2 +
                    (self.ip_vr_from1 - self.ip_vr_to1) * self.Gmutual2 -
                    (self.ip_vi_from1 - self.ip_vi_to1) * self.Bmutual2)

        Ir_to1 = -Ir_from1
        Ir_to2 = -Ir_from2

        return Ir_from1, Ir_to1, Ir_from2, Ir_to2

    def calc_imag_current_triplex(self):
        # Self admittance for conductors 1 and 2
        self.Gself1 = self.admittances[0][0].real
        self.Bself1 = self.admittances[0][0].imag
        self.Gself2 = self.admittances[1][1].real
        self.Bself2 = self.admittances[1][1].imag

        # Mutual admittance between conductors 1 and 2
        self.Gmutual1 = self.admittances[0][1].real
        self.Bmutual1 = self.admittances[0][1].imag
        self.Gmutual2 = self.admittances[1][0].real
        self.Bmutual2 = self.admittances[1][0].imag

        # Imaginary current calculation for each conductor
        Ii_from1 = ((self.ip_vi_from1 - self.ip_vi_to1) * self.Gself1 +
                    (self.ip_vr_from1 - self.ip_vr_to1) * self.Bself1 +
                    (self.ip_vi_from2 - self.ip_vi_to2) * self.Gmutual1 +
                    (self.ip_vr_from2 - self.ip_vr_to2) * self.Bmutual1)

        Ii_from2 = ((self.ip_vi_from2 - self.ip_vi_to2) * self.Gself2 +
                    (self.ip_vr_from2 - self.ip_vr_to2) * self.Bself2 +
                    (self.ip_vi_from1 - self.ip_vi_to1) * self.Gmutual2 +
                    (self.ip_vr_from1 - self.ip_vr_to1) * self.Bmutual2)

        Ii_to1 = -Ii_from1
        Ii_to2 = -Ii_from2

        return Ii_from1, Ii_to1, Ii_from2, Ii_to2


# Line where we have a collection of unbalanced phases (or a neutral wire) with admittance effects across wires.


class UnbalancedLine():

    def __init__(self,
                 network_model,
                 impedances,
                 shunt_admittances,
                 from_element,
                 to_element,
                 length,
                 phases="ABC",
                 ampacities=[]):
        self.lines: typing.List[UnbalancedLinePhase]
        self.lines = []

        self.impedances = np.array(impedances)
        if not (self.impedances.shape == (3, 3) or self.impedances.shape == (2, 2) or self.impedances.shape == (1, 1)):
            raise Exception(
                "incorrect impedances matrix size, expected a square matrix at most size 3 by 3")

        # Convert the per-meter impedance values to absolute, based on line length (in meters)
        self.impedances *= length

        try:
            self.admittances = calcInverse(self.impedances)
        except Exception:
            try:
                self.admittances = np.linalg.inv(self.impedances)
            except Exception:
                raise Exception(
                    "Transmission line was provided with a noninvertible matrix")

        #if not np.allclose(np.dot(self.impedances, self.admittances), np.identity(le309ggn(phases))):
        #    raise Exception(
        #        "np.linalg.inv was unable to find a good inverse to the impedance matrix")

        # Convert the per-meter shunt admittance values to absolute, based on line length (in meters)
        if shunt_admittances is None:
            self.shunt_admittances = None
        elif np.count_nonzero(shunt_admittances) == 0:
            self.shunt_admittances = None
        else:
            self.shunt_admittances = np.array(shunt_admittances)
            self.shunt_admittances *= length
            self.shunt_admittances = removeZeros(self.shunt_admittances)

        # print("Line", from_element, to_element, "impedances", self.impedances, "admittances", self.admittances, "shunt admittances", self.shunt_admittances)
        self.from_element = from_element
        self.to_element = to_element
        self.length = length
        self.network_model = network_model
        self.ampacities = ampacities
        self.phases = phases

        # print("Line", from_element, to_element, "impedances", self.impedances, "admittances", self.admittances, "shunt admittances", self.shunt_admittances)

        # TODO: This already creates all the lines you need to worry about.
        # for phase in phases:
        #    self.lines.append(UnbalancedLinePhase(self.from_element, self.to_element, phase, phases))
        for phase in phases:
            line_phase = UnbalancedLinePhase(
                self.from_element,
                self.to_element,
                phase,
                phases,
                self.admittances,
                self.shunt_admittances,
                self.network_model)
            self.lines.append(line_phase)
            # print(f"Phase {phase} is connected from {line_phase.from_element} to {line_phase.to_element}")

    def get_connections(self):
        for line in self.lines:
            from_bus, to_bus = line.get_nodes(self.network_model)
            yield (from_bus, to_bus)
