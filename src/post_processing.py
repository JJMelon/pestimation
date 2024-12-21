import matplotlib.pyplot as plt
from tabulate import tabulate
import numpy as np
import csv
from pyomo.environ import value
from models.components.measurement import get_transformer_phase

def get_phase_labels(phases):
    if set(phases) <= {'A', 'B', 'C'}:
        return ['A', 'B', 'C']
    elif set(phases) <= {'1', '2'}:
        return ['1', '2', None]
    else:
        raise ValueError("Unexpected phase labels")
########################### Power #########################
def transformer_power_to_csv(model, network, filename = "transformer_power.csv"):
    # Save transformer secondary power and voltage magnitude to CSV (to generate pseudo-meas)
    transformer_powers = {}
    for xfmr in network.transformers:
        ir_sec_value = value(model.xfmr_ir_sec_list[xfmr.FullName])
        ii_sec_value = value(model.xfmr_ii_sec_list[xfmr.FullName])

        # TODO: handle all types of xfmr connection
        vr_sec_value = value(xfmr.vr_a)
        vi_sec_value = value(xfmr.vi_a)
        if xfmr.name not in transformer_powers:
            transformer_powers[xfmr.name] = {}

        # Use phase info
        phase = get_transformer_phase(xfmr)
        P = ir_sec_value * vr_sec_value + ii_sec_value * vi_sec_value
        Q = ir_sec_value * vi_sec_value - ii_sec_value * vr_sec_value
        V = np.sqrt(vr_sec_value**2 + vi_sec_value**2)
        transformer_powers[xfmr.name][phase] = (P, Q, V)

    # Write the voltage profile to CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["transformer_name", "P_A", "Q_A", "voltA_mag", "P_B", "Q_B", "voltB_mag", "P_C", "Q_C", "voltC_mag"])

        for xfmr, vals in transformer_powers.items():
            row = [xfmr]
            phase_labels = vals.keys()

            for phase in phase_labels:
                if phase in vals:
                    p, q, v = vals[phase]
                    row.extend([f"{p:.6f}", f"{q:.6f}", f"{v:.6f}"])
                else:
                    row.extend(['0.000000', '0.000000', '0.000000'])
            writer.writerow(row)
    print(f"Transformer Data saved to {filename}.")

# Save the constant load power in a CSV (only need the network)
def load_power_to_csv(network, filename="load_profile.csv"):
    load_powers = {}

    for load in network.loads:
        if load.load_num not in load_powers:
            load_powers[load.load_num] = {}

        load_powers[load.load_num][load.phase] = (load.P, load.Q)
    # Write powers to CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["load_name", "P_A", "Q_A", "P_B", "Q_B", "P_C", "Q_C"])
        for load, powers in load_powers.items():
            row = [load]
            phase_labels = get_phase_labels(powers.keys())

            for phase in phase_labels:
                if phase in powers:
                    P_A, P_Q = powers[phase]
                    row.extend([f"{P_A:.6f}", f"{P_Q:.6f}"])
                else:
                    row.extend(['0.000000', '0.000000'])
            writer.writerow(row)

    print(f"Load Powers saved to {filename}.")


########################### Estimated Parameters #########################
# TODO: handle arbitrary parameters
def save_param_estimates_to_csv(network, model, filename = "estimated_params.csv"):

    # Write the voltage profile to CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["param_name", "value"])
        for param_name in list(model.estimated_line_set):
            estimate = value(model.estimated_line[param_name])
            row = []
            row.extend([f"{param_name}", f"{estimate:.6f}"])
            writer.writerow(row)

    print(f"Voltage profile saved to {filename}.")

########################### Voltage #########################

def voltage_profiles_to_csv(model, network, filename="voltage_profile.csv"):
    voltage_profile = {}

    # Retrieve bus and phase information from the network
    for bus in network.buses:
        vr_value = value(model.ipopt_vr_list[bus.NodeFullName])
        vi_value = value(model.ipopt_vi_list[bus.NodeFullName])

        if bus.NodeName not in voltage_profile:
            voltage_profile[bus.NodeName] = {}

        # Use the phase information directly from the bus
        for phase in bus.NodePhase:
            voltage_profile[bus.NodeName][phase] = (vr_value, vi_value)

    # Write the voltage profile to CSV
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["node_name", "voltA_real", "voltA_imag", "voltB_real", "voltB_imag", "voltC_real", "voltC_imag"])

        for node, voltages in voltage_profile.items():
            row = [node]
            phase_labels = get_phase_labels(voltages.keys())

            for phase in phase_labels:
                if phase in voltages:
                    vr, vi = voltages[phase]
                    row.extend([f"{vr:.6f}", f"{vi:.6f}"])
                else:
                    row.extend(['0.000000', '0.000000'])
            writer.writerow(row)

    print(f"Voltage profile saved to {filename}.")

def plot_voltage_profiles(model, bus, num_buses, num_slack, num_xfmr, num_load):
    # Node Voltage Results
    voltage_profile = []
    for bus in network.buses:
        voltage_profile.append(
            [bus.bus_name, model.ipopt_vr_list[bus.NodeFullName].value, model.ipopt_vi_list[bus.NodeFullName].value])
    print(tabulate(voltage_profile, headers=["Index", "Vr", "Vi"]))

    # # Slack Voltage Results
    # slack_voltage_profile = []
    # for ii in range(num_slack):
    #     slack_voltage_profile.append(
    #         [ii, model.slack_vr_list[ii].value, model.slack_vi_list[ii].value])
    # print(tabulate(slack_voltage_profile, headers=[
    #       "Index", "Vslack_r", "Vslack_i"]))

    # # Transformer Voltage Results
    # transformer_voltage_profile = []
    # for ii in range(num_xfmr):
    #     transformer_voltage_profile.append(
    #         [ii, model.xfmr_ir_pri_list[ii].value, model.xfmr_ii_pri_list[ii].value,
    #          model.xfmr_vr_aux_list[ii].value, model.xfmr_vi_aux_list[ii].value])
    # print(tabulate(transformer_voltage_profile, headers=[
    #       "Index", "Ir_pri", "Ii_pri", "Vr_aux", "Vi_aux"]))

    # # Infeasibility Currents
    # infeasibility_profile = []
    # for ii in range(num_load):
    #     infeasibility_profile.append(
    #         [ii, model.infeasi_ir_list[ii].value, model.infeasi_ii_list[ii].value])
    # print(tabulate(infeasibility_profile, headers=[
    #       "Index", "Ir_infeas", "Ii_infeas"]))

    # # Plotting Phase Voltages if Applicable
    # indices = list(range(1, num_buses + 1))
    # phase_A_values, phase_B_values, phase_C_values = [], [], []

    # for ii in range(0, num_buses, 3):
    #     phase_A_values.append(model.ipopt_vi_list[ii].value)
    # for ii in range(1, num_buses, 3):
    #     phase_B_values.append(model.ipopt_vi_list[ii].value)
    # for ii in range(2, num_buses, 3):
    #     phase_C_values.append(model.ipopt_vi_list[ii].value)

    # if phase_A_values and phase_B_values and phase_C_values:  # Check if phases are populated
    #     plt.figure(figsize=(10, 6))
    #     plt.plot(indices[:len(phase_A_values)],
    #              phase_A_values, '-o', label='Phase A')
    #     plt.plot(indices[:len(phase_B_values)],
    #              phase_B_values, '-p', label='Phase B')
    #     plt.plot(indices[:len(phase_C_values)],
    #              phase_C_values, '-d', label='Phase C')
    #     plt.xlabel('Bus number', fontsize=14)
    #     plt.ylabel('Voltage Value (Imaginary Component)', fontsize=14)
    #     plt.title('Voltage Profiles for Each Phase')
    #     plt.legend()
    #     plt.grid(True)
    #     plt.tight_layout()
    #     plt.show()


def plot_voltage_profiles_L1(model, num_buses, num_slack, num_xfmr, num_load):
    # Node Voltage Results
    voltage_profile = []
    for ii in range(num_buses):
        voltage_profile.append(
            [ii, model.ipopt_vr_list[ii].value, model.ipopt_vi_list[ii].value])
    print(tabulate(voltage_profile, headers=["Index", "Vr", "Vi"]))

    # Slack Voltage Results
    slack_voltage_profile = []
    for ii in range(num_slack):
        slack_voltage_profile.append(
            [ii, model.slack_vr_list[ii].value, model.slack_vi_list[ii].value])
    print(tabulate(slack_voltage_profile, headers=[
          "Index", "Vslack_r", "Vslack_i"]))

    # Transformer Voltage Results
    transformer_voltage_profile = []
    for ii in range(num_xfmr):
        transformer_voltage_profile.append(
            [ii, model.xfmr_ir_pri_list[ii].value, model.xfmr_ii_pri_list[ii].value,
             model.xfmr_vr_aux_list[ii].value, model.xfmr_vi_aux_list[ii].value])
    print(tabulate(transformer_voltage_profile, headers=[
          "Index", "Ir_pri", "Ii_pri", "Vr_aux", "Vi_aux"]))

    # Infeasibility Currents for L1 Norm
    infeasibility_profile = []
    for ii in range(num_load):
        # For L1, sum the positive and negative infeasibilities to get the total infeasibility
        ir_infeas_total = abs(
            model.pos_infeasi_ir_list[ii].value) + abs(model.neg_infeasi_ir_list[ii].value)
        ii_infeas_total = abs(
            model.pos_infeasi_ii_list[ii].value) + abs(model.neg_infeasi_ii_list[ii].value)
        infeasibility_profile.append([ii, ir_infeas_total, ii_infeas_total])

    print(tabulate(infeasibility_profile, headers=[
          "Index", "Ir_infeas", "Ii_infeas"]))


def tap_values(model, num_xfmr):
    # Initializing lists to store transformer IDs and their tap values
    xfmr_ids = range(num_xfmr)
    tap_values = [model.xfmr_tap_list[i].value for i in xfmr_ids]

    # Print tap values
    print("Transformer Tap Values:")
    for xfmr_id, tap_value in zip(xfmr_ids, tap_values):
        print(f"Transformer {xfmr_id}: Tap Value = {tap_value}")

    # plt.figure(figsize=(10, 6))
    # plt.bar(xfmr_ids, tap_values, color='skyblue')
    # plt.xlabel('Transformer ID', fontsize=14)
    # plt.ylabel('Tap Value', fontsize=14)
    # plt.title('Transformer Tap Values', fontsize=16)
    # plt.grid(axis='y', linestyle='--')
    # plt.tight_layout()
    # plt.show()


def print_infeasibility_currents(model, num_load):
    # Initializing lists to store infeasibility currents norms for each phase and their bus indices
    infeasibility_currents_norms = []
    bus_indices = range(num_load)

    # Calculating the norm of infeasibility currents for each bus
    for ii in bus_indices:
        ir_infeas = model.infeasi_ir_list[ii].value if ii in model.infeasi_ir_list else 0
        ii_infeas = model.infeasi_ii_list[ii].value if ii in model.infeasi_ii_list else 0
        norm = np.sqrt(ir_infeas**2 + ii_infeas**2)
        infeasibility_currents_norms.append(norm)

    # Printing infeasibility currents norms
    print("Infeasibility Currents Norms:")
    for bus_idx, norm in zip(bus_indices, infeasibility_currents_norms):
        print(f"Bus {bus_idx}: Infeasibility Current Norm = {norm}")


def print_infeasibility_currents_L1(model, num_load):
    # Initializing lists to store infeasibility currents norms for each phase and their bus indices
    infeasibility_currents_norms = []
    bus_indices = range(num_load)

    # Calculating the L1 norm of infeasibility currents for each bus
    for ii in bus_indices:
        pos_ir_infeas = model.pos_infeasi_ir_list[ii].value if ii in model.pos_infeasi_ir_list else 0
        neg_ir_infeas = model.neg_infeasi_ir_list[ii].value if ii in model.neg_infeasi_ir_list else 0
        pos_ii_infeas = model.pos_infeasi_ii_list[ii].value if ii in model.pos_infeasi_ii_list else 0
        neg_ii_infeas = model.neg_infeasi_ii_list[ii].value if ii in model.neg_infeasi_ii_list else 0

        # Summing of absolute values of positive and negative infeasibility currents for L1 norm
        norm = pos_ir_infeas + neg_ir_infeas + pos_ii_infeas + neg_ii_infeas
        infeasibility_currents_norms.append(norm)

    # Printing infeasibility currents norms
    print("Infeasibility Currents Norms:")
    for bus_idx, norm in zip(bus_indices, infeasibility_currents_norms):
        print(f"Bus {bus_idx}: Infeasibility Current Norm = {norm}")


def plot_infeasibility_currents(model, num_load):
    num_buses = num_load // 3
    bus_indices = range(num_buses)

    norms_phase_A = []
    norms_phase_B = []
    norms_phase_C = []

    # Calculating norms for each phase
    for ii in bus_indices:
        idx_a, idx_b, idx_c = ii*3, ii*3+1, ii*3+2

        # Phase A
        ir_infeas_A = model.infeasi_ir_list[idx_a].value if idx_a in model.infeasi_ir_list else 0
        ii_infeas_A = model.infeasi_ii_list[idx_a].value if idx_a in model.infeasi_ii_list else 0
        norms_phase_A.append(np.sqrt(ir_infeas_A**2 + ii_infeas_A**2))

        # Phase B
        ir_infeas_B = model.infeasi_ir_list[idx_b].value if idx_b in model.infeasi_ir_list else 0
        ii_infeas_B = model.infeasi_ii_list[idx_b].value if idx_b in model.infeasi_ii_list else 0
        norms_phase_B.append(np.sqrt(ir_infeas_B**2 + ii_infeas_B**2))

        # Phase C
        ir_infeas_C = model.infeasi_ir_list[idx_c].value if idx_c in model.infeasi_ir_list else 0
        ii_infeas_C = model.infeasi_ii_list[idx_c].value if idx_c in model.infeasi_ii_list else 0
        norms_phase_C.append(np.sqrt(ir_infeas_C**2 + ii_infeas_C**2))

    # Plotting
    plt.figure(figsize=(12, 8))
    width = 0.2  # Width of the bars
    plt.bar(np.array(list(bus_indices)) - width,
            norms_phase_A, width, label='Phase A', color='red')
    plt.bar(bus_indices, norms_phase_B, width, label='Phase B', color='green')
    plt.bar(np.array(list(bus_indices)) + width, norms_phase_C,
            width, label='Phase C', color='blue')

    plt.xlabel('Bus Index', fontsize=14)
    plt.ylabel('Infeasibility Current Norm [A]', fontsize=14)
    plt.title('Infeasibility Currents Norms per Phase', fontsize=16)
    plt.xticks(bus_indices)
    plt.legend()
    plt.grid(axis='y', linestyle='--')
    plt.tight_layout()
    plt.show()


def plot_infeasibility_currents_L1(model, num_load):
    num_buses = num_load // 3
    bus_indices = range(num_buses)

    norms_phase_A = []
    norms_phase_B = []
    norms_phase_C = []

    # Calculating norms for each phase
    for ii in bus_indices:
        idx_a, idx_b, idx_c = ii*3, ii*3+1, ii*3+2

        # Phase A
        pos_ir_infeas_A = model.pos_infeasi_ir_list[idx_a].value if idx_a in model.pos_infeasi_ir_list else 0
        neg_ir_infeas_A = model.neg_infeasi_ir_list[idx_a].value if idx_a in model.neg_infeasi_ir_list else 0
        pos_ii_infeas_A = model.pos_infeasi_ii_list[idx_a].value if idx_a in model.pos_infeasi_ii_list else 0
        neg_ii_infeas_A = model.neg_infeasi_ii_list[idx_a].value if idx_a in model.neg_infeasi_ii_list else 0
        norms_phase_A.append(
            pos_ir_infeas_A + neg_ir_infeas_A + pos_ii_infeas_A + neg_ii_infeas_A)

        # Phase B
        pos_ir_infeas_B = model.pos_infeasi_ir_list[idx_b].value if idx_b in model.pos_infeasi_ir_list else 0
        neg_ir_infeas_B = model.neg_infeasi_ir_list[idx_b].value if idx_b in model.neg_infeasi_ir_list else 0
        pos_ii_infeas_B = model.pos_infeasi_ii_list[idx_b].value if idx_b in model.pos_infeasi_ii_list else 0
        neg_ii_infeas_B = model.neg_infeasi_ii_list[idx_b].value if idx_b in model.neg_infeasi_ii_list else 0
        norms_phase_B.append(
            pos_ir_infeas_B + neg_ir_infeas_B + pos_ii_infeas_B + neg_ii_infeas_B)

        # Phase C
        pos_ir_infeas_C = model.pos_infeasi_ir_list[idx_c].value if idx_c in model.pos_infeasi_ir_list else 0
        neg_ir_infeas_C = model.neg_infeasi_ir_list[idx_c].value if idx_c in model.neg_infeasi_ir_list else 0
        pos_ii_infeas_C = model.pos_infeasi_ii_list[idx_c].value if idx_c in model.pos_infeasi_ii_list else 0
        neg_ii_infeas_C = model.neg_infeasi_ii_list[idx_c].value if idx_c in model.neg_infeasi_ii_list else 0
        norms_phase_C.append(
            pos_ir_infeas_C + neg_ir_infeas_C + pos_ii_infeas_C + neg_ii_infeas_C)

    # Plotting
    plt.figure(figsize=(12, 8))
    width = 0.2  # Width of the bars
    plt.bar(np.array(list(bus_indices)) - width,
            norms_phase_A, width, label='Phase A', color='red')
    plt.bar(bus_indices, norms_phase_B, width, label='Phase B', color='green')
    plt.bar(np.array(list(bus_indices)) + width, norms_phase_C,
            width, label='Phase C', color='blue')

    plt.xlabel('Bus Index', fontsize=14)
    plt.ylabel('Infeasibility Current Norm [A]', fontsize=14)
    plt.title('Infeasibility Currents Norms per Phase', fontsize=16)
    plt.xticks(bus_indices)
    plt.legend()
    plt.grid(axis='y', linestyle='--')
    plt.tight_layout()
    plt.show()


def plot_total_infeasibility_currents(model, num_load):
    total_infeasibility_currents = []
    bus_set_indices = []

    for ii in range(0, num_load, 3):
        # Calculating the norms of infeasibility currents for each phase
        norm_a = np.sqrt(
            model.infeasi_ir_list[ii].value**2 + model.infeasi_ii_list[ii].value**2)
        norm_b = np.sqrt(
            model.infeasi_ir_list[ii+1].value**2 + model.infeasi_ii_list[ii+1].value**2)
        norm_c = np.sqrt(
            model.infeasi_ir_list[ii+2].value**2 + model.infeasi_ii_list[ii+2].value**2)

        # Sum of norms for all three phases to get the total infeasibility for the bus set
        total_infeasibility = norm_a + norm_b + norm_c
        total_infeasibility_currents.append(total_infeasibility)

        bus_set_indices.append(ii // 3)

    total_infeasibility_currents = np.array(total_infeasibility_currents)
    bus_set_indices = np.array(bus_set_indices)

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.bar(bus_set_indices, total_infeasibility_currents, color='green')
    plt.xlabel('Bus Set Index', fontsize=14)
    plt.ylabel('Total Infeasibility Current [A]', fontsize=14)
    plt.title('Total Infeasibility Currents for Three-Phase Bus Sets', fontsize=16)
    plt.grid(True)
    plt.tight_layout()
    # plt.savefig('infeasibility_currents_bar_chart.png', dpi=300)
    plt.show()


def plot_total_infeasibility_currents_L1(model, num_load):
    total_infeasibility_currents = []
    bus_set_indices = []

    for ii in range(0, num_load, 3):
        # Calculating the L1 norms of infeasibility currents for each phase
        norm_a = model.pos_infeasi_ir_list[ii].value + model.neg_infeasi_ir_list[ii].value + \
            model.pos_infeasi_ii_list[ii].value + \
            model.neg_infeasi_ii_list[ii].value
        norm_b = model.pos_infeasi_ir_list[ii+1].value + model.neg_infeasi_ir_list[ii+1].value + \
            model.pos_infeasi_ii_list[ii+1].value + \
            model.neg_infeasi_ii_list[ii+1].value
        norm_c = model.pos_infeasi_ir_list[ii+2].value + model.neg_infeasi_ir_list[ii+2].value + \
            model.pos_infeasi_ii_list[ii+2].value + \
            model.neg_infeasi_ii_list[ii+2].value

        # Sum of norms for all three phases to get the total infeasibility for the bus set
        total_infeasibility = norm_a + norm_b + norm_c
        total_infeasibility_currents.append(total_infeasibility)

        bus_set_indices.append(ii // 3)

    total_infeasibility_currents = np.array(total_infeasibility_currents)
    bus_set_indices = np.array(bus_set_indices)

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.bar(bus_set_indices, total_infeasibility_currents, color='green')
    plt.xlabel('Bus Set Index', fontsize=14)
    plt.ylabel('Total Infeasibility Current [A]', fontsize=14)
    plt.title('Total Infeasibility Currents for Three-Phase Bus Sets', fontsize=16)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def save_all_voltages_to_csv(network, model, csv_filename):
    data_to_write = []

    # Collecting voltages for all buses
    for bus in network.buses:
        vr = value(model.ipopt_vr_list[bus.NodeFullName])
        vi = value(model.ipopt_vi_list[bus.NodeFullName])
        vmag = (vr**2 + vi**2)**0.5
        data_to_write.append([bus.int_bus_id, vr, vi])

    # Writing the collected data to the CSV file
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Bus ID', 'Vreal', 'Vimag'])
        writer.writerows(data_to_write)
