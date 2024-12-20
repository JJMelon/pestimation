from itertools import count
from sympy import symbols
from models.components.bus import GROUND, Bus
from models.wellknownvariables import Vr_from, Vi_from, Vr_to, Vi_to


class Generator:
    _ids = count(0)

    def __init__(self,
                 bus: Bus,
                 P,
                 Vset,
                 Qmax,
                 Qmin,
                 Pmax,
                 Pmin,
                 Qinit,
                 RemoteBus,
                 RMPCT,
                 gen_type):
        """Initialize an instance of a generator in the power grid.

        Args:
            Bus (int): the bus number where the generator is located.
            P (float): the current amount of active power the generator is providing.
            Vset (float): the voltage setpoint that the generator must remain fixed at.
            Qmax (float): maximum reactive power
            Qmin (float): minimum reactive power
            Pmax (float): maximum active power
            Pmin (float): minimum active power
            Qinit (float): the initial amount of reactive power that the generator is supplying or absorbing.
            RemoteBus (int): the remote bus that the generator is controlling
            RMPCT (float): the percent of total MVAR required to hand the voltage at the controlled bus
            gen_type (str): the type of generator
        """

        self.id = self._ids.__next__()

        self.bus = bus
        self.P = -P
        self.Vset = Vset

        self.Qinit = -Qinit

        self.Qmax = -Qmax
        self.Qmin = -Qmin


    def get_connections(self):
        return []


