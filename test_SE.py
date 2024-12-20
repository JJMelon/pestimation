import pandas as pd
import numpy as np
import json
import os
import subprocess
import pickle

# Generate measurement files with added noise
def generate_noisy_measurements(true_voltages_csv, loads_csv, true_transformer_data_csv, output_folder, N, std_values):
    # Get true values from previous powerflow solution
    true_voltages = pd.read_csv(true_voltages_csv)
    load_data = pd.read_csv(loads_csv)
    true_transformer_data = pd.read_csv(true_transformer_data_csv)
    phases = ["A", "B", "C"]
    # Number of Realizations for each noise distribution
    K = 100
    pmu_buses = ['node1']
    transformer_sensors = ['transformer:23']
    ami_loads = ['load4']
    cable_box_buses = ['node2']

    # dict with err as key, list of realizations filenames as value
    files_dict = {}
    for i, std in enumerate(std_values):
        json_files = []
        for realization in range(K):
            json_data = []

            # From powerflow voltages
            #   -> Add noisy PMU measurement only at Node1
            #   -> Add noisy cablebox measurement at Node2
            for _, row in true_voltages.iterrows():
                row_dict = row.to_dict()
                if row_dict['node_name'] in pmu_buses:
                    for phase in phases:
                        for component in ["real", "imag"]:
                        # Dont add imag component to reference bus! keep angle 0
                            if component == "imag" and row_dict['node_name'] == 'node1':
                                pass
                            else:
                                field = f"volt{phase}_{component}"
                                if field in row_dict:
                                    # normalized gaussian noise with standard deviation of err
                                    std_absolute = abs(row_dict[field])*std
                                    noise = np.random.normal(0, std_absolute)
                                    row_dict[field] = round(row_dict[field] + noise, 2)

                    row_dict["noise_std"] = std_absolute
                    row_dict["phases"] = phases
                    row_dict["device_type"] = "injection_pmu"
                    json_data.append(row_dict)

                # Add cable box at each of 3 phases in line
                if row_dict['node_name'] in cable_box_buses:
                    for phase in phases:
                        # Add new object for each phase
                        new_dict = {"node_name": row_dict['node_name'], \
                                "phase": phase,\
                                "device_type": "cable_box"}

                        # Add measurement value
                        vi = row_dict[f"volt{phase}_imag"]
                        vr = row_dict[f"volt{phase}_real"]
                        vmag = np.sqrt(vi**2 + vr**2)
                        std_absolute = vmag*std
                        noise = np.random.normal(0, std_absolute)
                        new_dict["Vmag"] = round(vmag + noise, 2)
                        new_dict['noise_std'] = std_absolute
                        json_data.append(new_dict)

            # From transformer data -> Add noisy transformer measurements at secondary
            for _, row in true_transformer_data.iterrows():
                row_dict = row.to_dict()
                if row_dict['transformer_name'] in transformer_sensors:
                    for phase in phases:
                        for meas in ["P", "Q", "volt"]:
                            if meas == "volt":
                                field = f"volt{phase}_mag"
                                std_absolute_v = abs(row_dict[field])*std
                                noise = np.random.normal(0, std_absolute_v)
                                row_dict[field] = round(row_dict[field] + noise, 2)
                            else:
                                field = f"{meas}_{phase}"
                                std_absolute_pq = abs(row_dict[field])*std
                                noise = np.random.normal(0, std_absolute_pq)
                                row_dict[field] = round(row_dict[field] + noise, 2)

                    row_dict["noise_std_pq"] = std_absolute_pq
                    row_dict["noise_std_v"] = std_absolute_v
                    row_dict["phases"] = phases
                    row_dict["device_type"] = "transformer_sensor"
                    json_data.append(row_dict)

            # From load data -> Add noisy AMI measurements
            for _, row in load_data.iterrows():
                row_dict = row.to_dict()
                if row_dict['load_name'] in ami_loads:
                    for phase in phases:
                        # Add new object for each phase
                        new_dict = {"load_name": row_dict['load_name'], \
                                "phase": phase,\
                                "device_type": "ami"}
                        for meas in ["P", "Q"]:
                            field = f"{meas}_{phase}"
                            if field in row_dict:
                                std_absolute = abs(row_dict[field])*std
                                noise = np.random.normal(0, std_absolute)
                                new_dict[meas] = round(row_dict[field] + noise, 2)
                                new_dict['noise_std'] = std_absolute
                        json_data.append(new_dict)

            json_file = os.path.join(output_folder, f"measurements_std{std:.5f}_{realization}.json")
            with open(json_file, 'w') as f:
                json.dump(json_data, f, indent=4)

            json_files.append(json_file)

            # only need one perfect measurement file
            if std == 0:
                break
        files_dict[std] = json_files

    return files_dict

# Run State Estimation for each file of measurement data and calculate the MNAE
def run_and_evaluate(files_dict, true_csv, output_folder):
    error_results = []

    # run for each realization of each value of error standard deviation
    for error, file_names in files_dict.items():
        nmae_vals = np.zeros(len(file_names))
        nae_param_vals = np.zeros(len(file_names))
        for i, json_file in enumerate(file_names):
            print(f"Parameter Estimation for {json_file} running...", flush=True)
            subprocess.run(
                    ["python", "Combined-txd/src/main.py", "-estimate", f"-measurements={json_file}"],
                    stdout=subprocess.PIPE,    # Suppress standard output
                    stderr=subprocess.PIPE     # Suppress error output)
                    )
            # Load values from csv files
            estimated_csv = "voltage_profile.csv"
            estimated_params_csv = "estimated_params.csv"
            true_params_csv = "true_params.csv"
            true_params = pd.read_csv(true_params_csv, header = 0)
            estimated_params = pd.read_csv(estimated_params_csv, header = 0)
            true_data = pd.read_csv(true_csv, header=0)
            estimated_data = pd.read_csv(estimated_csv, header=0)

            # Ensure that the columns are numeric
            true_data = true_data.apply(pd.to_numeric, errors='coerce')
            estimated_data = estimated_data.apply(pd.to_numeric, errors='coerce')
            true_params = true_params.apply(pd.to_numeric, errors='coerce')
            estimated_params = estimated_params.apply(pd.to_numeric, errors='coerce')

            column = "voltA_real" # column we will calculate error in
            abs_error = (true_data[column] - estimated_data[column]).abs()
            abs_param_error = (true_params["value"] - estimated_params["value"]).abs()

            # Normalize parameter estimation error
            nae_param = abs_param_error / abs(true_params['value'])
            #TODO: Only compare values we are estimating

            # Find the maximum absolute error and its index
            max_error = abs_error.max()
            max_error_index = abs_error.idxmax()

            # Normalize the maximum error by the true value at the index where max error occurs
            if true_data[column][max_error_index] != 0:
                nmae = max_error / abs(true_data[column][max_error_index])
            else:
                nmae = float("inf")  # Handle division by zero if the true value is zero

            nmae_vals[i] = nmae * 100
            nae_param_vals[i] = nae_param[0] * 100

        # Results are in % units
        error_results.append({
            "std": error * 100,
            "nmae_voltA_real": nmae_vals,
            "nae_G_self_1-2": nae_param_vals
        })

    return error_results


# Generate different measurement data files, run state estimation then plot mnae
def different_noise_std_test() -> list:
    voltages = 'voltage_profile_true.csv'
    loads = 'load_profile_true.csv'
    transformer_data = 'transformer_power_true.csv'
    output_folder = 'Combined-txd/output'
    N = 5  # Number of different values for std

    std_values = np.array([0, 0.001, 0.002, 0.003, 0.004, 0.005])  # Generate values of standard deviation

    os.makedirs(output_folder, exist_ok=True)

    # Generate JSON files with noise
    json_files = generate_noisy_measurements(voltages, loads, transformer_data, output_folder, N, std_values)

    # Run state estimation and evaluate NMAE
    nmae_results = run_and_evaluate(json_files, voltages, output_folder)

    return nmae_results

# Perform state estimation with PMU at node1, transformer sensor at secondary, and 3 AMI
# --> Perfect Measurements should result in less than 1% error!
def test_transformer_sensor():
    true_values_file = 'voltage_profile_true.csv'  # Path to input CSV file used to generate
    output_folder = 'output'  # Folder for output JSON files and voltage profiles
    measurement_file = 'Combined-txd/measurement_devices_1pmu_1sensor_3ami.json'

    # Run main.py and evalute NMSE
    nmse = run_and_evaluate([measurement_file], true_values_file, output_folder)
    print(f'Maximum Normalized Absolute Error for voltA_real: {nmse[0]["nmae_voltA_real"]}')

if __name__ == "__main__":
    errors = different_noise_std_test()
    # Save data to a pickle file
    with open('error_results.pkl', 'wb') as f:
        pickle.dump(errors, f)



