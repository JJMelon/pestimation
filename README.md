# Three-Phase Distribution Infeasibility Analysis

Features:
Three-Phase Distribution Analysis: Perform detailed simulations on three-phase distribution networks.
Distribution Infeasibility Analysis: Identify and analyze infeasibility issues within distribution systems.
Parameter Estimation: Estimate selected parameters in three-phase distribution networks using measurement data.


## Quick Start

Assuming you cloned the repository and you are in the root of the repository, first install all of the python modules as listed in `requirements.txt`:

```
python -m pip install -r ./requirements.txt
```

This library can be executed with arguments to target a particular three-phase network case

Note! This may take a while to execute the first time as everything gets derived/compiled. Subsequent executions should be much faster.

```
python src/main.py
```

## Files and Modules Overview

-`main.py`: Executes the main workflow, which includes loading the network configuration from a `.glm` file, setting power flow parameters, and initiating the power flow simulation.

- `logic/`: Hosts the core Python modules that define the network structure, manage load configurations, and specify operational settings essential for the simulation.
- `ipopt_setup.py`:Integrates the components, sets up and executes the IPOPT optimization solver, applying predefined constraints and the objective function.
- `models/components`: Contains detailed definitions and models for various electrical system components such as buses, transformers, and power lines, crucial for accurate simulation.
- `post_processing`: Provides functionality for visualizing results and further processing of data, such as plotting voltage profiles and exporting them for analysis.

## Customizing the Simulation

Modify the simulation to meet specific requirements by altering configurations and parameters:

- **Simulation Settings**: Adjust settings in the `PowerFlowSettings` class within `main.py`. Parameters such as maximum iterations, voltage limits, and convergence criteria can be fine-tuned to optimize the simulation results.
- **Optimization Parameters**: Modify solver settings in `ipopt_setup.py`, including tolerance and iteration limits. T
