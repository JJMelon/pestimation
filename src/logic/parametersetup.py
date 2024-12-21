# Assigns IPOPT variables to parameters being estimated
# 1. Line lengths -> sets all phase conductances on each line being estimated to the same variable (assumes all phase wires have identical conductances)
# 2. Transformer tap ratio: TODO
from logic.network.networkmodel import DxNetworkModel

# creates dictionary of lines keyed by name and adds it to network model
def setup_lines_dict(network: DxNetworkModel):
    network.lines_dict = {line.name: line for line in network.lines}


def assign_line_params(network: DxNetworkModel, model):
    for line_name in model.estimated_line_set:
        line = network.lines_dict[line_name]
        for phase_line in line.lines:
            phase_line.length = model.estimated_line[line_name]

