class PowerFlowSettings:
    def __init__(
        self,
        tolerance=1E-05,
        max_iters=50,
        voltage_limiting=False,
        debug=False,
        flat_start=False,
        tx_stepping=False,
        dump_matrix=False,
        device_control=True,
        load_factor=None,
        enable_line_capacitance=True,
        enable_cable_current_bounds=False,
        enable_bus_voltage_bounds=True,
        infeasibility_analysis=False,
        L1_norm=True,
        L2_norm=False,
    ) -> None:
        self.tolerance = tolerance
        self.max_iters = max_iters
        self.voltage_limiting = voltage_limiting
        self.debug = debug
        self.flat_start = flat_start
        self.tx_stepping = tx_stepping
        self.dump_matrix = dump_matrix
        self.device_control = device_control
        self.load_factor = load_factor
        self.enable_line_capacitance = enable_line_capacitance
        self.enable_cable_current_bounds = enable_cable_current_bounds
        self.enable_bus_voltage_bounds = enable_bus_voltage_bounds
        self.infeasibility_analysis = infeasibility_analysis
        self.L1_norm = L1_norm
        self.L2_norm = L2_norm



