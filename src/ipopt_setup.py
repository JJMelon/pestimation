from pyomo.environ import Var, Set, ConcreteModel, ConstraintList, Reals, NonNegativeReals, Objective, minimize, SolverFactory, value, expr, Param
from models.components import slack
from models.components import transformer
from models.components import bus
from pyomo.environ import Constraint
from logic.parametersetup import *
import matplotlib.pyplot as plt
import numpy as np
import sys
import csv
from post_processing import *
from models.components.transformer import Transformer
from models.components.center_tap_transformer import CenterTapTransformer
from models.components import regulator
from tabulate import tabulate
from itertools import count
import os


def ipopt_setup(network, settings, input_file):

    num_buses = len(network.buses)
    num_slack = len(network.slack)
    num_xfmr = len(network.transformers)
    num_fuse = len(network.fuses)
    num_switch = len(network.switches)
    num_load = len(network.loads)
    num_regulator = len(network.regulators)
    num_capacitor = len(network.capacitors)
    num_transformer_sensor = len(network.transformer_sensors)
    num_ami = len(network.amis)
    num_cable_box = len(network.cable_boxes)

    print("Number of buses: ", num_buses)
    print("Number of slack buses: ", num_slack)
    print("Number of transformers: ", num_xfmr)
    print("Number of loads: ", num_load)
    print("Number of regulators: ", num_regulator)  # Print the number of regulators
    print("Number of fuses: ", num_fuse)  # Print the number of fuses
    print("Number of switches: ", num_switch)  # Print the number of switches
    print("Number of capacitors: ", num_capacitor) # Print the number of capacitors

    if settings.estimation:
        num_pmu = len(network.pmus)
        print("Number of Transformer Sensors: ", num_transformer_sensor)
        print("Number of PMUs: ", num_pmu) # Print the number of capacitors
        print("Number of AMIs: ", num_ami)
        print("Numer of Cable Boxes: ", num_cable_box)

    num_infeasible = num_buses - num_slack

    print("________________________________________________________________________________________________________________________________")

    # num_CT = 0
    # num_normal_transformers = 0

    # for transformer in network.transformers:
    #     if isinstance(transformer, CenterTapTransformer):
    #         num_CT += 1
    #     elif isinstance(transformer, Transformer):
    #         num_normal_transformers += 1

    # print("Number of Center Tap Transformers:", num_CT)
    # print("Number of Normal Transformers:", num_normal_transformers)

    num_normal_transformers = Transformer._ids.__next__()
    num_CT = CenterTapTransformer._ids.__next__()
    print("Number of Center Tap Transformers:", num_CT)
    print("Number of Normal Transformers:", num_normal_transformers)

    num_triplex_loads = 0
    num_normal_loads = 0

    for load in network.loads:
        if load.triplex_phase is not None:
            num_triplex_loads += 1
        else:
            num_normal_loads += 1

    print("Number of triplex Loads:", num_triplex_loads)
    print("Number of three-phase Loads:", num_normal_loads)

    num_triplex_lines = 0
    num_normal_lines = 0
    for element in network.lines:
        for line in element.lines:
            if '1' in line.phases:
                num_triplex_lines += 1
            else:
                num_normal_lines += 1
    print("Number of triplex Lines:", num_triplex_lines)
    print("Number of three-phase Lines:", num_normal_lines)

    print("________________________________________________________________________________________________________________________________")
    num_eq = 2*num_buses + 2*num_slack + 6*num_normal_transformers + 6*num_CT +\
          4*num_regulator + 2*num_fuse + 2*num_switch
    print("Expected number of ipopt equations: ", num_eq)

    model = ConcreteModel()
    model.name = "dist_opt"

    ########################### Initializing ##########################
    # Defining Parameters for current limits
    model.current_limit = Param(initialize=300)  # Initializing with the value 300
    model.current_limit_triplex = Param(initialize=100)  # Initializing with the value 100
    vr_init = 10
    vi_init = 10
    ir_init = 10
    ii_init = 10

    # Create list of bus names
    bus_names = [bus.NodeFullName for bus in network.buses]
    model.bus_names_set = Set(initialize=bus_names)
    model.ipopt_vr_list = Var(model.bus_names_set, initialize=vr_init)
    model.ipopt_vi_list = Var(model.bus_names_set, initialize=vi_init)

    # Create list of slack names
    slack_names = [slack.bus.NodeFullName for slack in network.slack]
    model.slack_names_set = Set(initialize=slack_names)
    model.slack_vr_list = Var(model.slack_names_set, initialize=vr_init)
    model.slack_vi_list = Var(model.slack_names_set, initialize=vi_init)

    # Transformer
    # Create list of transformer names
    xfmr_names = [f'{xfmr.FullName}' for xfmr in network.transformers if not isinstance(xfmr, CenterTapTransformer)]
    model.xfmr_name_set = Set(initialize=xfmr_names)
    model.xfmr_vr_aux_list = Var(model.xfmr_name_set, initialize=vr_init)
    model.xfmr_vi_aux_list = Var(model.xfmr_name_set, initialize=vi_init)
    model.xfmr_ir_pri_list = Var(model.xfmr_name_set, initialize=ir_init)
    model.xfmr_ii_pri_list = Var(model.xfmr_name_set, initialize=ii_init)
    model.xfmr_ir_sec_list = Var(model.xfmr_name_set, initialize=ir_init)
    model.xfmr_ii_sec_list = Var(model.xfmr_name_set, initialize=ii_init)

    # CT
    # Create list of CT names
    CT_names = [f'{xfmr.FullName}' for xfmr in network.transformers if isinstance(xfmr, CenterTapTransformer)]
    model.CT_names_set = Set(initialize=CT_names)
    model.CT_ir_pri_list = Var(model.CT_names_set, initialize=ir_init)
    model.CT_ii_pri_list = Var(model.CT_names_set, initialize=ii_init)
    model.CT_ir_sec1_list = Var(model.CT_names_set, initialize=ir_init)
    model.CT_ii_sec1_list = Var(model.CT_names_set, initialize=ii_init)
    model.CT_ir_sec2_list = Var(model.CT_names_set, initialize=ir_init)
    model.CT_ii_sec2_list = Var(model.CT_names_set, initialize=ii_init)

    # Regulator
    model.reg_ir_pri_list = Var(range(num_regulator))
    model.reg_ii_pri_list = Var(range(num_regulator))
    model.reg_vr_aux_list = Var(range(num_regulator))
    model.reg_vi_aux_list = Var(range(num_regulator))

    # Fuse
    model.fuse_ir_list = Var(range(num_fuse))
    model.fuse_ii_list = Var(range(num_fuse))

    # Switch
    model.switch_ir_list = Var(range(num_switch))
    model.switch_ii_list = Var(range(num_switch))

    # AMI P, Q measurement variables
    # Create list of load names
    load_names = [load.FullName for load in network.loads]
    model.load_names_set = Set(initialize=load_names)
    if settings.estimation:
        model.load_p_list = Var(model.load_names_set)
        model.load_q_list = Var(model.load_names_set)

    # Parameters to Estimate
    if settings.estimation:
        estimated_line_names = ['overhead_line:12', 'overhead_line:34']
        max_line_length = 1e4 # maximum reasonable line length (m) -> bound feasible space
        model.estimated_line_set = Set(initialize=estimated_line_names)
        model.estimated_line = Var(model.estimated_line_set, bounds=(0,max_line_length))
        setup_lines_dict(network)
        assign_line_params(network, model)

    # Infeasibility current L2
    if settings.infeasibility_analysis:
        if settings.L2_norm:
            model.infeasi_ir_list = Var(range(num_infeasible))
            model.infeasi_ii_list = Var(range(num_infeasible))
        if settings.L1_norm:
                model.pos_infeasi_ir_list = Var(range(num_infeasible), within=NonNegativeReals)
                model.neg_infeasi_ir_list = Var(range(num_infeasible), within=NonNegativeReals)
                model.pos_infeasi_ii_list = Var(range(num_infeasible), within=NonNegativeReals)
                model.neg_infeasi_ii_list = Var(range(num_infeasible), within=NonNegativeReals)


################################## Constraint ##############################
    model.cons = ConstraintList()

    KCL_equations_real = dict()
    KCL_equations_imag = dict()

    # Transformer
    xfmr_ir_aux_constraint = dict()
    xfmr_ii_aux_constraint = dict()
    xfmr_vr_pri_constraint = dict()
    xfmr_vi_pri_constraint = dict()
    xfmr_ir_sec_constraint = dict()
    xfmr_ii_sec_constraint = dict()


    # Regulator
    reg_vr_aux_constraint = dict()
    reg_vi_aux_constraint = dict()
    reg_vr_pri_constraint = dict()
    reg_vi_pri_constraint = dict()

    #CT
    Vr_sec_cons1_dict = dict()
    Vi_sec_cons1_dict = dict()
    Vr_sec_cons2_dict = dict()
    Vi_sec_cons2_dict = dict()
    Ir_pri_cons_dict = dict()
    Ii_pri_cons_dict = dict()

    #Slack
    vr_slack_constraint = dict()
    vi_slack_constraint = dict()

    #Fuse
    vr_fuse_constraint = dict()
    vi_fuse_constraint = dict()

    #Switch
    vr_switch_constraint = dict()
    vi_switch_constraint = dict()
    for bus in network.buses:
        bus.create_ipopt_bus_vars(model)

    for load in network.loads:
        P = load.get_P()
        Q = load.get_Q()
        node_index = load.get_node_index()

    # For Lines
    for ele in network.lines:
        # For each phase in the line
        for line in ele.lines:

            if not line.is_triplex:
                line.create_ipopt_vars(model, network.buses)

            elif line.is_triplex:
                line.create_ipopt_vars_triplex(model, network.buses)

            line.add_to_eqn_list(KCL_equations_real,
                                 KCL_equations_imag)


            if settings.infeasibility_analysis:
                ######################## Current limit ########################################
                if settings.enable_cable_current_bounds:
                    # Use model.current_limit and model.current_limit_triplex
                    if not line.is_triplex:

                        Ir_from, Ir_to = line.calc_real_current()
                        Ii_from, Ii_to = line.calc_imag_current()
                        model.cons.add((Ir_from**2 + Ii_from**2) <= model.current_limit**2)
                        model.cons.add((Ir_to**2 + Ii_to**2) <= model.current_limit**2)

                    elif line.is_triplex:
                        Ir_from1, Ir_to1, Ir_from2, Ir_to2 = line.calc_real_current_triplex()
                        Ii_from1, Ii_to1, Ii_from2, Ii_to2 = line.calc_imag_current_triplex()
                        model.cons.add((Ir_from1**2 + Ii_from1**2) <= model.current_limit_triplex**2)
                        model.cons.add((Ir_to1**2 + Ii_to1**2) <= model.current_limit_triplex**2)
                        model.cons.add((Ir_from2**2 + Ii_from2**2) <= model.current_limit_triplex**2)
                        model.cons.add((Ir_to2**2 + Ii_to2**2) <= model.current_limit_triplex**2)
                ######################## Current limit ########################################




    # For Slack
    for slack_model in network.slack:
        slack_model.assign_ipopt_vars(model)
        slack_model.add_to_eqn_list(KCL_equations_real, KCL_equations_imag,
                                    vr_slack_constraint, vi_slack_constraint)

    # For Transformer
    for transformer in network.transformers:
        #
        if isinstance(transformer, CenterTapTransformer):
            transformer.assign_CT_ipopt_vars(model)
            transformer.add_to_eqn_list_CT(KCL_equations_real, KCL_equations_imag,
                           Vr_sec_cons1_dict, Vi_sec_cons1_dict, Vr_sec_cons2_dict, Vi_sec_cons2_dict,
                           Ir_pri_cons_dict, Ii_pri_cons_dict)
        #
        elif isinstance(transformer, Transformer):
            transformer.assign_xfmr_ipopt_vars(model)
            transformer.add_to_eqn_list(KCL_equations_real, KCL_equations_imag,
                                        xfmr_ir_aux_constraint, xfmr_ii_aux_constraint,
                                        xfmr_vr_pri_constraint, xfmr_vi_pri_constraint, xfmr_ir_sec_constraint, xfmr_ii_sec_constraint)

    # For Load
    if settings.estimation: # Replace loads with AMI power injection
        for ami in network.amis:
            #TODO: handle Triplex loads with AMI
            ami.assign_ipopt_vars(model)
            ami.add_to_eqn_list(KCL_equations_real, KCL_equations_imag)
    else:
        for load_model in network.loads:
            if hasattr(load_model, 'triplex_phase') and load_model.triplex_phase is not None:
                # print(load_model.phase)
                load_model.create_ipopt_vars_triplex(model)
                load_model.add_to_eqn_list_triplex(
                    KCL_equations_real, KCL_equations_imag)
            else:
                load_model.create_ipopt_vars(model)
                load_model.add_to_eqn_list(
                    KCL_equations_real, KCL_equations_imag)

    # For capacitors
    for capacitor_model in network.capacitors:
        capacitor_model.create_ipopt_vars(model)
        capacitor_model.add_to_eqn_list(KCL_equations_real, KCL_equations_imag)

    # For fuse
    for fuse_model in network.fuses:
        fuse_model.assign_ipopt_vars(model)
        fuse_model.add_to_eqn_list(
            KCL_equations_real, KCL_equations_imag, vr_fuse_constraint, vi_fuse_constraint)

    # For switch
    for switch_model in network.switches:
        switch_model.assign_ipopt_vars(model)
        switch_model.add_to_eqn_list(
            KCL_equations_real, KCL_equations_imag, vr_switch_constraint, vi_switch_constraint)

    # For Regulators
    for reg in network.regulators:
        reg.assign_reg_ipopt_vars(model)
        reg.add_to_eqn_list(KCL_equations_real, KCL_equations_imag,
                            reg_vr_aux_constraint, reg_vi_aux_constraint,
                            reg_vr_pri_constraint, reg_vi_pri_constraint)

    # Add measurement constraints
    if settings.estimation:
        for pmu in network.pmus:
            pmu.assign_ipopt_vars(model)
            pmu.add_to_eqn_list(KCL_equations_real, KCL_equations_imag)

        for sensor in network.transformer_sensors:
            sensor.assign_ipopt_vars(model)
            sensor.calculate_constraints()

        for cable_box in network.cable_boxes:
            cable_box.assign_ipopt_vars(model)

    # Adding the infeasibility current to the KCL equations
    if settings.infeasibility_analysis:
        for ii in network.slack:
            slack_node_info = ii.slack_node_info()

        if settings.L2_norm:

            # Adding to the real part of the KCL equations
            for jj in range(num_infeasible):
                if jj != slack_node_info:
                    if jj in KCL_equations_real:
                        KCL_equations_real[jj] += (-model.infeasi_ir_list[jj])
                    else:
                        KCL_equations_real[jj] = (-model.infeasi_ir_list[jj])
            # Adding to the imaginary part of the KCL equations
            for jj in range(num_infeasible):
                if jj != slack_node_info:
                    if jj in KCL_equations_imag:
                        KCL_equations_imag[jj] += (-model.infeasi_ii_list[jj])
                    else:
                        KCL_equations_imag[jj] = (-model.infeasi_ii_list[jj])

        elif settings.L1_norm:

            for jj in range(num_infeasible):
                if jj != slack_node_info:
                    # For the real part of the KCL equations
                    if jj in KCL_equations_real:
                        KCL_equations_real[jj] += (-model.pos_infeasi_ir_list[jj] +
                                                   model.neg_infeasi_ir_list[jj])
                    else:
                        KCL_equations_real[jj] = (-model.pos_infeasi_ir_list[jj] +
                                                  model.neg_infeasi_ir_list[jj])
                    # For the imaginary part of the KCL equations
                    if jj in KCL_equations_imag:
                        KCL_equations_imag[jj] += (-model.pos_infeasi_ii_list[jj] +
                                                   model.neg_infeasi_ii_list[jj])
                    else:
                        KCL_equations_imag[jj] = (-model.pos_infeasi_ii_list[jj] +
                                                  model.neg_infeasi_ii_list[jj])

    ################################# Voltage limit###############################
    if settings.infeasibility_analysis:
        if settings.enable_bus_voltage_bounds:

            def read_bus_voltages(csv_filename):
                bus_voltages = {}
                with open(csv_filename, mode='r') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        bus_ID = row['Bus ID']
                        bus_voltages[bus_ID] = {
                            'VR': float(row['Vreal']),
                            'VI': float(row['Vimag'])
                        }
                return bus_voltages

            use_bus_voltage_csv = True

            def apply_voltage_constraints(model, network, bus_voltages, special_bus_ids, special_up, special_lo):
                for i, bus in enumerate(network.buses):
                    if use_bus_voltage_csv:
                        if str(bus.int_bus_id) in bus_voltages:
                            voltages = bus_voltages[str(bus.int_bus_id)]
                            vr_ref = voltages['VR']
                            vi_ref = voltages['VI']

                            V_ref_squared = vr_ref**2 + vi_ref**2
                            bus_ID = bus.int_bus_id

                            V_squared = model.ipopt_vr_list[i]**2 + model.ipopt_vi_list[i]**2

                            if bus_ID in special_bus_ids:
                                up = special_up
                                lo = special_lo
                            else:
                                up = 1.05
                                lo = 0.95

                            model.cons.add(V_squared <= V_ref_squared * up**2)
                            model.cons.add(V_squared >= V_ref_squared * lo**2)

            special_bus_ids = []
            special_up = 1.2285
            special_lo = 0.8120

            base_name = os.path.basename(os.path.dirname(input_file))
            output_directory = "pf_voltages"
            os.makedirs(output_directory, exist_ok=True)

            csv_filename = os.path.join(output_directory, f"{base_name}_voltages.csv")
            if use_bus_voltage_csv:
                bus_voltages = read_bus_voltages(csv_filename)
            else:
                bus_voltages = None

            apply_voltage_constraints(model, network, bus_voltages, special_bus_ids, special_up, special_lo)
    ################################# Voltage limit###############################

################### Objective Function ###################
    # Estimation Weight
    def get_weight(std: float):
        if std == 0:
            return 1
        else:
            return (1/std)**2

    #  Objective Function with Infeasibility Analysis #
    if settings.infeasibility_analysis:
        if settings.L1_norm:
            model.objective = Objective(
                expr=sum(model.pos_infeasi_ir_list[i] + model.neg_infeasi_ir_list[i] for i in range(num_infeasible)) +
                sum(model.pos_infeasi_ii_list[i] + model.neg_infeasi_ii_list[i]
                    for i in range(num_infeasible)),
                sense=minimize
            )
        elif settings.L2_norm:
            model.objective = Objective(expr=sum(0.5 * (model.infeasi_ir_list[i]**2) for i in model.infeasi_ir_list) + sum(
                0.5 * (model.infeasi_ii_list[i]**2) for i in model.infeasi_ii_list), sense=minimize)

    #  Objective Function with WLS Estimation #
    elif settings.estimation:
        model.objective = Objective(
            expr=(
                sum(get_weight(pmu.var) * (pmu.Ir_G**2 + pmu.Ii_G**2) for pmu in network.pmus) +
                sum(get_weight(sensor.var_pq) * (sensor.P_error**2 + sensor.Q_error**2) + \
                    get_weight(sensor.var_v) * sensor.V_error**2 for sensor in network.transformer_sensors) +
                sum(get_weight(ami.var) * (ami.P_error**2 + ami.Q_error**2) for ami in network.amis) +
                sum(get_weight(cable_box.var) * cable_box.error**2 for cable_box in network.cable_boxes)),
            sense = minimize
        )

    #  Objective Function without Infeasibility Analysis #
    else:
        def obj_expression(model):
            return 1
        model.objective = Objective(rule=obj_expression, sense=minimize)

################### Objective Function ###################
    #################### Constraints ####################
    model.KCL_real_constraints = ConstraintList()
    model.KCL_imag_constraints = ConstraintList()

    # Slack Constraint List
    model.vr_slack_constraint = ConstraintList()
    model.vi_slack_constraint = ConstraintList()

    # Xfmr Constraint List
    model.xfmr_ir_aux_constraint = ConstraintList()
    model.xfmr_ii_aux_constraint = ConstraintList()
    model.xfmr_vr_pri_constraint = ConstraintList()
    model.xfmr_vi_pri_constraint = ConstraintList()
    model.xfmr_ir_sec_constraint = ConstraintList()
    model.xfmr_ii_sec_constraint = ConstraintList()

    # Reg Constraint List
    model.reg_vr_aux_constraint = ConstraintList()
    model.reg_vi_aux_constraint = ConstraintList()
    model.reg_vr_pri_constraint = ConstraintList()
    model.reg_vi_pri_constraint = ConstraintList()

    # CT Constraint List
    model.Vr_sec_cons1 = ConstraintList()
    model.Vi_sec_cons1 = ConstraintList()
    model.Vr_sec_cons2 = ConstraintList()
    model.Vi_sec_cons2 = ConstraintList()
    model.Ir_pri_cons = ConstraintList()
    model.Ii_pri_cons = ConstraintList()

    # Fuse and Switch Constraint List
    model.vr_fuse_constraint = ConstraintList()
    model.vi_fuse_constraint = ConstraintList()
    model.vr_switch_constraint = ConstraintList()
    model.vi_switch_constraint = ConstraintList()

    model.voltage_constraints = ConstraintList()

    num_KCL_real_constraints = 0
    num_KCL_imag_constraints = 0
    num_vr_slack_constraints = 0
    num_vi_slack_constraints = 0
    num_xfmr_ir_aux_constraints = 0
    num_xfmr_ii_aux_constraints = 0
    num_xfmr_vr_pri_constraints = 0
    num_xfmr_vi_pri_constraints = 0
    num_xfmr_ir_sec_constraints = 0
    num_xfmr_ii_sec_constraints = 0
    num_reg_vr_aux_constraints = 0
    num_reg_vi_aux_constraints = 0
    num_reg_vr_pri_constraints = 0
    num_reg_vi_pri_constraints = 0
    num_Vr_sec_cons1_constraints = 0
    num_Vi_sec_cons1_constraints = 0
    num_Vr_sec_cons2_constraints = 0
    num_Vi_sec_cons2_constraints = 0
    num_Ir_pri_cons_constraints = 0
    num_Ii_pri_cons_constraints = 0
    num_vr_fuse_constraints = 0
    num_vi_fuse_constraints = 0
    num_vr_switch_constraints = 0
    num_vi_switch_constraints = 0

    for key, equation in KCL_equations_real.items():
        model.KCL_real_constraints.add(expr=equation == 0)
        num_KCL_real_constraints += 1

    for key, equation in KCL_equations_imag.items():
        model.KCL_imag_constraints.add(expr=equation == 0)
        num_KCL_imag_constraints += 1

    for key, equation in vr_slack_constraint.items():
        model.vr_slack_constraint.add(expr=equation == 0)
        num_vr_slack_constraints += 1

    for key, equation in vi_slack_constraint.items():
        model.vi_slack_constraint.add(expr=equation == 0)
        num_vi_slack_constraints += 1

    # Transformer
    for key, equation in xfmr_ir_aux_constraint.items():
        model.xfmr_ir_aux_constraint.add(expr=equation == 0)
        num_xfmr_ir_aux_constraints += 1

    for key, equation in xfmr_ii_aux_constraint.items():
        model.xfmr_ii_aux_constraint.add(expr=equation == 0)
        num_xfmr_ii_aux_constraints += 1

    for key, equation in xfmr_vr_pri_constraint.items():
        model.xfmr_vr_pri_constraint.add(expr=equation == 0)
        num_xfmr_vr_pri_constraints += 1

    for key, equation in xfmr_vi_pri_constraint.items():
        model.xfmr_vi_pri_constraint.add(expr=equation == 0)
        num_xfmr_vi_pri_constraints += 1

    for key, equation in xfmr_ir_sec_constraint.items():
        model.xfmr_ir_sec_constraint.add(expr=equation == 0)
        num_xfmr_ir_sec_constraints += 1

    for key, equation in xfmr_ii_sec_constraint.items():
        model.xfmr_ii_sec_constraint.add(expr=equation == 0)
        num_xfmr_ii_sec_constraints += 1


    # Regulator
    for key, equation in reg_vr_aux_constraint.items():
        model.reg_vr_aux_constraint.add(expr=equation == 0)
        num_reg_vr_aux_constraints += 1

    for key, equation in reg_vi_aux_constraint.items():
        model.reg_vi_aux_constraint.add(expr=equation == 0)
        num_reg_vi_aux_constraints += 1

    for key, equation in reg_vr_pri_constraint.items():
        model.reg_vr_pri_constraint.add(expr=equation == 0)
        num_reg_vr_pri_constraints += 1

    for key, equation in reg_vi_pri_constraint.items():
        model.reg_vi_pri_constraint.add(expr=equation == 0)
        num_reg_vi_pri_constraints += 1

    # CT
    for key, equation in Vr_sec_cons1_dict.items():
        model.Vr_sec_cons1.add(expr=equation == 0)
        num_Vr_sec_cons1_constraints += 1

    for key, equation in Vi_sec_cons1_dict.items():
        model.Vi_sec_cons1.add(expr=equation == 0)
        num_Vi_sec_cons1_constraints += 1

    for key, equation in Vr_sec_cons2_dict.items():
        model.Vr_sec_cons2.add(expr=equation == 0)
        num_Vr_sec_cons2_constraints += 1

    for key, equation in Vi_sec_cons2_dict.items():
        model.Vi_sec_cons2.add(expr=equation == 0)
        num_Vi_sec_cons2_constraints += 1

    for key, equation in Ir_pri_cons_dict.items():
        model.Ir_pri_cons.add(expr=equation == 0)
        num_Ir_pri_cons_constraints += 1

    for key, equation in Ii_pri_cons_dict.items():
        model.Ii_pri_cons.add(expr=equation == 0)
        num_Ii_pri_cons_constraints += 1

    # Fuse and Switch
    for key, equation in vr_fuse_constraint.items():
        model.vr_fuse_constraint.add(expr=equation == 0)
        num_vr_fuse_constraints += 1

    for key, equation in vi_fuse_constraint.items():
        model.vi_fuse_constraint.add(expr=equation == 0)
        num_vi_fuse_constraints += 1

    for key, equation in vr_switch_constraint.items():
        model.vr_switch_constraint.add(expr=equation == 0)
        num_vr_switch_constraints += 1

    for key, equation in vi_switch_constraint.items():
        model.vi_switch_constraint.add(expr=equation == 0)
        num_vi_switch_constraints += 1

    print("Number of KCL real constraints:", num_KCL_real_constraints)
    print("Number of KCL imaginary constraints:", num_KCL_imag_constraints)
    print("Number of VR slack constraints:", num_vr_slack_constraints)
    print("Number of VI slack constraints:", num_vi_slack_constraints)
    print("Number of XFMR VR auxiliary constraints:", num_xfmr_ir_aux_constraints)
    print("Number of XFMR VI auxiliary constraints:", num_xfmr_ii_aux_constraints)
    print("Number of XFMR VR primary constraints:", num_xfmr_vr_pri_constraints)
    print("Number of XFMR VI primary constraints:", num_xfmr_vi_pri_constraints)
    print("Number of XFMR IR secondary constraints:", num_xfmr_ir_sec_constraints)
    print("Number of XFMR II secondary constraints:", num_xfmr_ii_sec_constraints)
    print("Number of regulator VR auxiliary constraints:", num_reg_vr_aux_constraints)
    print("Number of regulator VI auxiliary constraints:", num_reg_vi_aux_constraints)
    print("Number of regulator VR primary constraints:", num_reg_vr_pri_constraints)
    print("Number of regulator VI primary constraints:", num_reg_vi_pri_constraints)
    print("Number of CT VR secondary constraints 1:", num_Vr_sec_cons1_constraints)
    print("Number of CT VI secondary constraints 1:", num_Vi_sec_cons1_constraints)
    print("Number of CT VR secondary constraints 2:", num_Vr_sec_cons2_constraints)
    print("Number of CT VI secondary constraints 2:", num_Vi_sec_cons2_constraints)
    print("Number of CT IR primary constraints:", num_Ir_pri_cons_constraints)
    print("Number of CT II primary constraints:", num_Ii_pri_cons_constraints)
    print("Number of VR fuse constraints:", num_vr_fuse_constraints)
    print("Number of VI fuse constraints:", num_vi_fuse_constraints)
    print("Number of VR switch constraints:", num_vr_switch_constraints)
    print("Number of VI switch constraints:", num_vi_switch_constraints)

    total_constraints = (
        num_KCL_real_constraints +
        num_KCL_imag_constraints +
        num_vr_slack_constraints +
        num_vi_slack_constraints +
        num_xfmr_ir_aux_constraints +
        num_xfmr_ii_aux_constraints +
        num_xfmr_vr_pri_constraints +
        num_xfmr_vi_pri_constraints +
        num_xfmr_ir_sec_constraints +
        num_xfmr_ii_sec_constraints +
        num_reg_vr_aux_constraints +
        num_reg_vi_aux_constraints +
        num_reg_vr_pri_constraints +
        num_reg_vi_pri_constraints +
        num_Vr_sec_cons1_constraints +
        num_Vi_sec_cons1_constraints +
        num_Vr_sec_cons2_constraints +
        num_Vi_sec_cons2_constraints +
        num_Ir_pri_cons_constraints +
        num_Ii_pri_cons_constraints +
        num_vr_fuse_constraints +
        num_vi_fuse_constraints +
        num_vr_switch_constraints +
        num_vi_switch_constraints
    )

    print("Total number of constraints:", total_constraints)

    tolerance = 1e-6  # settings["tolerance"]
    max_iter = 10000  # settings["max_iter"]
    Obj_scal_factor = 1
    flag_tee = True

    # model.pprint()
    print("*********************************************************************************************************************")
    def print_model_statistics(model):
        total_vars = sum(len(vars) for vars in model.component_objects(Var, active=True))
        print(f"Total number of variables: {total_vars}")
        # print("Variable names:")
        # for var in model.component_objects(Var, active=True):
        #     var_object = getattr(model, var.name)
        #     for index in var_object:
        #         print(var_object[index].name)

        total_constraints = sum(len(constraint_list) for constraint_list in model.component_objects(Constraint, active=True))
        print(f"Total number of constraints: {total_constraints}")
    print_model_statistics(model)
    print("*********************************************************************************************************************")
    if True:
        solver = SolverFactory('ipopt')
        solver.options['tol'] = tolerance
        if settings.estimation:
            solver.options['print_level'] = 0
        solver.options["max_iter"] = max_iter
        solver.options['print_frequency_iter'] = 1
        solver.options["print_user_options"] = "yes"
        solver.options["print_info_string"] = "yes"
        solver.options["honor_original_bounds"] = "yes"
        solver.options["mu_init"] = 1e-2
        solver.options["obj_scaling_factor"] = Obj_scal_factor
        solver.options['linear_solver'] = "ma57"
        solver.options['hsllib'] = "/usr/local/lib/libcoinhsl.so"
        pyomo_options = {"tee": flag_tee}
        results = solver.solve(model, **pyomo_options)
        # model.solutions.store_to(results)
    print("Solver status:", results.solver.status)
    print("Solver termination condition:",
          results.solver.termination_condition)
    print("Objective value:", value(model.objective))

    voltage_profiles_to_csv(model, network)
    transformer_power_to_csv(model, network)
    load_power_to_csv(network)

    if settings.estimation:
        save_param_estimates_to_csv(network, model)

    if settings.infeasibility_analysis:
        pass
    else:
        base_name = os.path.basename(os.path.dirname(input_file))
        output_directory = "pf_voltages"
        os.makedirs(output_directory, exist_ok=True)

        csv_filename = os.path.join(output_directory, f"{base_name}_voltages.csv")

        save_all_voltages_to_csv(network, model, csv_filename)
        print(f"All bus voltages saved to {csv_filename}")

    # def save_all_buses_to_csv(network, filename):
    #     header = ["Bus Name", "Bus ID", "Phases", "Voltage Real", "Voltage Imaginary"]
    #     with open(filename, mode='w', newline='') as file:
    #         writer = csv.writer(file)

    #         writer.writerow(header)
    #         for bus in network.buses:
    #             writer.writerow([bus.NodeName, bus.int_bus_id, bus.NodePhase, bus.Vr_init, bus.Vi_init])
    # save_all_buses_to_csv(network, 'buses.csv')

    if settings.infeasibility_analysis:
        tolerance = 1e-5

        if settings.L2_norm:
            output_directory = "infeasibility_currents"
            os.makedirs(output_directory, exist_ok=True)

            csv_filename = os.path.join(output_directory, "infeasibility_L2.csv")
            save_infeasibility_L2_to_csv(network, model, csv_filename)
            print(f"Infeasibility currents for L2 norm saved to {csv_filename}.")

        elif settings.L1_norm:
            output_directory = "infeasibility_currents"
            os.makedirs(output_directory, exist_ok=True)

            csv_filename = os.path.join(output_directory, "infeasibility_L1.csv")
            save_infeasibility_L1_to_csv(network, model, csv_filename)
            print(f"Infeasibility currents for L1 norm saved to {csv_filename}.")
