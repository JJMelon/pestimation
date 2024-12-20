from __future__ import division
import math
from pyomo.environ import ConcreteModel, ConstraintList, Objective
from pyomo.environ import Var as PyomoVar
from models.components.bus import GROUND, Bus
from models.components.load import Load
from models.components.transformer import Transformer
from logic.network.networkmodel import DxNetworkModel
from itertools import count
import json

# TODO: handle all four connection types!
def get_transformer_phase(xfmr: Transformer) -> str:
    return xfmr.to_bus_pos.NodePhase


# Linear PMU Model for Injection at a Bus
class InjectionPMU:
    _ids = count(0)
    def __init__(self, bus: Bus, var: float, measurements: dict):
        """ Initialize a PMU object at a bus in the grid
        Args:
            Bus (int): the bus where the PMU is located
        """
        self.id = self._ids.__next__()
        self.bus = bus
        self.bus_idx = bus.int_bus_id
        self.var = var
        self.measurements = measurements

    # Assign bus voltage variables
    def assign_ipopt_vars(self, model):
        self.ipopt_vr = model.ipopt_vr_list[self.bus_idx]
        self.ipopt_vi = model.ipopt_vi_list[self.bus_idx]

    # Add KCL terms
    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag):
        Vr_bus = self.ipopt_vr
        Vi_bus = self.ipopt_vi
        Vr_pmu = self.measurements['V_real']
        Vi_pmu = self.measurements['V_imag']

        # Ohms Law for PMU with only voltage measurement, fixed G = 1 TODO: Add current injection
        self.Ir_G = (Vr_pmu - Vr_bus)
        self.Ii_G = (Vi_pmu - Vi_bus)

        if self.bus_idx in KCL_equations_real:
            KCL_equations_real[self.bus_idx] += self.Ir_G
        else:
            KCL_equations_real[self.bus_idx] = self.Ir_G

        if self.bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.bus_idx] += self.Ii_G
        else:
            KCL_equations_imag[self.bus_idx] = self.Ii_G

class TransformerSensor():
    _ids = count(0)
    def __init__(self, phase: str, transformer: Transformer, std_pq: float, std_v: float, measurements: dict):
        """ Initialize a transformer sensor object at one phase of a transformer
        Args:
            phase (str): Phase that sensor is installed on
            transformer (Transformer): the transformer object sensor is located at
            var (float): variance used for pseudo-measurements
            measurements (dict): dictionary of measurement values for this sensor
        """
        self.id = self._ids.__next__()
        self.transformer = transformer
        self.transformer_idx = transformer.id
        self.bus_idx = transformer.to_bus_pos.int_bus_id
        self.phase = phase
        self.var_pq = std_pq
        self.var_v = std_v
        self.measurements = measurements

    # Assign secondary TO bus voltage, and secondary current variables
    def assign_ipopt_vars(self, model):
        if self.transformer.is_wye_wye():
            self.vr = self.transformer.vr_a
            self.vi = self.transformer.vi_a
            self.ir = self.transformer.ir_a
            self.ii = self.transformer.ii_a
        elif selfformer.is_delta_wye():
            self.vr = self.transformer.vr_b
            self.vi = self.transformer.vi_b
            self.ir = self.transformer.ir_b
            self.ii = self.transformer.ii_b

    # Calculate measurement equation in terms of transformer vars
    def calculate_constraints(self):
        self.P_error = self.measurements['P'] - self.vr * self.ir - self.vi * self.ii
        self.Q_error = self.measurements['Q'] - self.vi * self.ir + self.vr * self.ii
        self.V_error = self.measurements['V']**2 - self.vr**2 - self.vi**2

class AMI():
    _ids = count(0)
    def __init__(self, phase:str , load: Load, var: float, measurements: dict):
        self.phase = phase
        self.id = self._ids.__next__()
        self.load = load
        self.var = var
        self.meas = measurements

    # Assign variables from load object and P, Q and error
    def assign_ipopt_vars(self, model):
        self.vr = model.ipopt_vr_list[self.load.from_bus_idx]
        self.vi = model.ipopt_vi_list[self.load.from_bus_idx]
        self.P = model.load_p_list[self.load.id]
        self.Q = model.load_q_list[self.load.id]
        self.P_error = self.meas['P'] - self.P
        self.Q_error = self.meas['Q'] - self.Q

    def cal_current(self):
        I_lr = ((self.P * self.vr) + (self.Q * self.vi)) / \
            (self.vr**2 + self.vi**2)
        I_li = ((self.P * self.vi) - (self.Q * self.vr)) / \
            (self.vr**2 + self.vi**2)
        return I_lr, I_li


    def add_to_eqn_list(self, KCL_equations_real, KCL_equations_imag):
        (I_lr, I_li) = self.cal_current()
        bus_idx = self.load.from_bus_idx
        if self.load.from_bus_idx in KCL_equations_real:
            KCL_equations_real[self.load.from_bus_idx] += I_lr
        else:
            KCL_equations_real[self.load.from_bus_idx] = I_lr
        if self.load.from_bus_idx in KCL_equations_imag:
            KCL_equations_imag[self.load.from_bus_idx] += I_li
        else:
            KCL_equations_imag[self.load.from_bus_idx] = I_li

class CableBox():
    _ids = count(0)
    def __init__(self, bus: Bus, Vmag: float, var: float):
        self.bus_id = bus.int_bus_id
        self.id = self._ids.__next__()
        self.phase = bus.NodePhase
        self.Vmag = Vmag
        self.var = var

    # Assign bus voltage variables and error equations
    def assign_ipopt_vars(self, model):
        self.ipopt_vr = model.ipopt_vr_list[self.bus_id]
        self.ipopt_vi = model.ipopt_vi_list[self.bus_id]
        self.error = (self.Vmag)**2 - (self.ipopt_vr)**2 - (self.ipopt_vi)**2


# Adds an injection PMU to the network model at a specific bus for each of the measured phases
def create_injection_pmu(device: dict, network_model: DxNetworkModel):
    # Check Phases of PMU and map bus_name to network bus ID

    for phase in device['phases']:
        # extract measurements for this phase
        meas_dict = dict()
        meas_dict['V_real'] = device['volt' + phase + '_real']
        meas_dict['V_imag'] = device['volt' + phase + '_imag']

        # TODO: add current injection
        #meas_dict['I_real'] = device['curr' + phase + '_real']
        #meas_dict['I_imag'] = device['curr' + phase + '_imag']

        # Find the correct phase parent bus in the network model
        parent_bus = network_model.bus_name_map[device['node_name'] + '_' + phase]

        # Create PMU object and add to network model
        injection_pmu = InjectionPMU(parent_bus, device['noise_std'], meas_dict)
        network_model.pmus.append(injection_pmu)


# Adds a sensor to the network model at a specific transformer secondary
def create_transformer_sensor(device: dict, network_model: DxNetworkModel):
    # Find the transformers in the network model
    xfmrs = [xfmr for xfmr in network_model.transformers if xfmr.name == device['transformer_name']]

    # Create a dictionary to identify transformer phases
    phase_xfmrs = {get_transformer_phase(xfmr): xfmr for xfmr in xfmrs}

    for phase in device['phases']:
        # extract measurements for this phase (P, Q, Vmag)
        meas_dict = dict()
        meas_dict['V'] = device['volt' + phase + '_mag']
        meas_dict['P'] = device['P_' + phase]
        meas_dict['Q'] = device['Q_' + phase]

        sensor = TransformerSensor(phase, phase_xfmrs[phase], device['noise_std_pq'], device['noise_std_v'], meas_dict)
        network_model.transformer_sensors.append(sensor)

def create_ami(device: dict, network_model: DxNetworkModel):
    meas_dict = dict()
    meas_dict['P'] = device['P']
    meas_dict['Q'] = device['Q']
    # Get load object at which AMI is attached
    phase = device['phase']
    load = next(
        (load for load in network_model.loads if load.load_num == device['load_name'] and load.phase == phase),
        None  # Default value if no match found
    )
    #loads = [load in network_model.loads where (load.load_num == device['name'] and load.phase == phase)]
    ami = AMI(phase, load, device['noise_std'], meas_dict)
    network_model.amis.append(ami)

def create_cable_box(device: dict, network_model: DxNetworkModel):
    bus = next(
        (bus for bus in network_model.buses if bus.NodeName == device['node_name'] and bus.NodePhase == device['phase']),
        None  # Default value if no match found
    )
    cable_box = CableBox(bus, device['Vmag'], device['noise_std'])
    network_model.cable_boxes.append(cable_box)

# Reads measurement device objects from JSON file and adds them to the network model
def add_measurement_devices(file_name: str, network_model: DxNetworkModel):
    with open(file_name, 'r') as file:
        devices = json.load(file)

    for device in devices:
        # Case on device type
        if device['device_type'] == "injection_pmu":
            create_injection_pmu(device, network_model)

        elif device['device_type'] == "transformer_sensor":
            create_transformer_sensor(device, network_model)

        elif device['device_type'] == "ami":
            create_ami(device, network_model)

        elif device['device_type'] == 'cable_box':
            create_cable_box(device, network_model)



