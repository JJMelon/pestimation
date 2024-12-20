from sympy import symbols
from models.components.bus import Bus
from models.wellknownvariables import Vr_from, Vi_from, Vr_to, Vi_to


class VoltageSource:
    def __init__(self, from_bus: Bus, to_bus: Bus, Vr_set, Vi_set) -> None:
        self.from_bus = from_bus
        self.to_bus = to_bus
        self.Vr_set = Vr_set
        self.Vi_set = Vi_set

    def get_connections(self):
        return [(self.from_bus, self.to_bus)]


#Glorified voltage source, just a better name for readability sometimes.
class CurrentSensor(VoltageSource):
    def __init__(self, from_bus: Bus, to_bus: Bus) -> None:
        VoltageSource.__init__(self, from_bus, to_bus, 0, 0)
