from logic.network.networkloader import NetworkLoader
from logic.network.networkmodel import NetworkModel, TxNetworkModel
from logic.powerflowsettings import PowerFlowSettings
from models.components.measurement import add_measurement_devices
from ipopt_setup import ipopt_setup
from logic.graphanalyzer import GraphAnalyzer

import argparse
import os
from pathlib import Path

settings = PowerFlowSettings(
    debug=False,
    max_iters=50,
    flat_start=False,
    tx_stepping=False,
    voltage_limiting=False,
    dump_matrix=False,
    load_factor=None,
    infeasibility_analysis=False
)

parser = argparse.ArgumentParser(description="Process estimation options and measurement file.")
# Define the -estimate argument as a boolean flag
parser.add_argument(
    '-estimate',
    action='store_true',
    help="Flag to indicate if estimation should be performed"
)

# Define the -measurements argument to accept a filename
parser.add_argument(
    '-measurements',
    type=str,
    help="Path to the JSON file with measurements",
    required=False
)

# Parse the arguments
args = parser.parse_args()

# Store the values in variables or use them directly
estimate_flag = args.estimate
measurements_file = args.measurements

# Print the values (or use them as needed in the script)
if estimate_flag:
    print("Estimation mode enabled.")
if measurements_file:
    print(f"Using measurements file: {measurements_file}")

# Enable Estimation
settings.estimation = estimate_flag

input_file = "test/data/three_phase/ieee_four_bus/node.glm" #12 buses
# input_file = "test/data/three_phase/ieee_four_bus_with_triplex/node.glm" # 19 buses (0.00% error)
# input_file = "test/data/three_phase/ieee_four_bus_switch/node.glm" #12 buses
# input_file = "test/data/three_phase/ieee_four_bus/balanced_stepdown_D-grY.glm" #12 buses
# input_file = "test/data/three_phase/ieee_four_bus_delta_delta_transformer/node.glm" #12 buses
# input_file = "test/data/three_phase/gc_12_47_1_no_reg/node.glm" #93 buses
# input_file = "test/data/three_phase/gc_12_47_1_no_cap/node_modified.glm" #99 buses - Does not run
# input_file = "test/data/three_phase/gc_12_47_1/node.glm" #96 buses
# input_file = "test/data/three_phase/ieee_four_bus_with_triplex/node.glm" #19 buses
# input_file = "test/data/three_phase/network_model_case1_342/node.glm" #4260 buses
# input_file = "test/data/three_phase/network_model_case1/node.glm" #4260 buses
# input_file = "test/data/three_phase/network_model_case2/node.glm" #4260 buses
# input_file = "test/data/three_phase/center_tap_xfmr/special_fixed.glm" #16 buses
#input_file = "test/data/three_phase/center_tap_xfmr/node.glm" #14 buses
# input_file = "test/data/three_phase/center_tap_xfmr_and_line_to_load/node.glm"  #20 buses (very good)
# input_file = "test/data/three_phase/center_tap_xfmr_and_single_line_to_load/node.glm" #16 buses
# input_file = "test/data/three_phase/center_tap_xfmr_and_triplex_line/node.glm" #20 buses
# input_file = "test/data/three_phase/center_tap_xfmr_and_triplex_load/node.glm"  #14 buses (very good)
# input_file = "test/data/three_phase/regulator_center_tap_xfmr/node.glm"
# input_file = "test/data/three_phase/r1_12_47_2/node.glm"  # 2297 buses (max 6% error)
# input_file = "test/data/three_phase/r1_12_47_3/node.glm"  # 181 buses (max 0.46%)
# input_file = "test/data/three_phase/r1_12_47_4/node.glm"  # 1196 buses (max 0.87%)
# input_file = "test/data/three_phase/r1_25_00_1/node.glm"  # 853 buses (max 6.57%)
# input_file = "test/data/three_phase/r2_35_00_1/node.glm"  # 4136 buses (max 1.76%)
# input_file = "test/data/three_phase/r3_12_47_1/node.glm"  # 4888 buses (max 2.50%)
# input_file = "test/data/three_phase/r3_12_47_2/node.glm"  # 866  buses (max 0.03%)
# input_file = "test/data/three_phase/r3_12_47_3/node.glm"  # 16546 buses (max 6.17%)
# input_file = "test/data/three_phase/r4_12_47_1/node.glm"  # 4478 buses (max 6.56%)
# input_file = "test/data/three_phase/r4_12_47_2/node.glm"  # 1719 buses (max 0.92)
# input_file = "test/data/three_phase/r4_25_00_1/node.glm"  # 1305 buses (max 0.34)
# input_file = "test/data/three_phase/r5_12_47_1/node.glm"  # 2002 buses (max 3.12)
# input_file = "test/data/three_phase/r5_12_47_2/node.glm"  # 2000 buses (max 1.59)
# input_file = "test/data/three_phase/r5_12_47_3/node.glm"  # 10807 buses (max 33.43)
# input_file = "test/data/three_phase/r5_12_47_4/node.glm"  # 2723 buses (max 3.15%)
# input_file = "test/data/three_phase/r5_12_47_5/node.glm"  # 4491 buses (max 4.98%)
# input_file = "test/data/three_phase/r5_25_00_1/node.glm"  # 4323 buses (max 0.97%)
# input_file = "test/data/three_phase/r5_35_00_1/node.glm"  # 2071 buses (max 0.78%)
# input_file = "test/data/three_phase/unbalanced_triplex_load/node.glm"
# input_file = "test/data/three_phase/triplex_load_class/node.glm"
#input_file = "test/data/three_phase/south_hero/node.glm" #20043 buses (max 1.17%)
# input_file = "south_hero_nosolar_noinverter.glm"
# input_file = "burton_hill_mod2.glm"
#input_file = "balanced_stepdown_D-D.glm"


network = NetworkLoader(settings).from_file(input_file)
if settings.estimation:
    add_measurement_devices(measurements_file, network)

ipopt_setup(network, settings, input_file)
