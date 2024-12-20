import numpy as np
from sympy import symbols
from itertools import count
from models.components.bus import Bus, GROUND
from models.wellknownvariables import tx_factor, Vr_from, Vi_from, Vr_to, Vi_to, Lr_from, Li_from, Lr_to, Li_to

class Line:
    _ids = count(0)

    def __init__(self,
                 from_bus: Bus,
                 to_bus: Bus,
                 r,
                 x,
                 b,
                 status,
                 rateA,
                 rateB,
                 rateC):

        self.id = self._ids.__next__()

        self.from_bus = from_bus
        self.to_bus = to_bus

        self.r = r
        self.x = x
        self.b = b

        self.G = r / (x ** 2 + r ** 2)
        self.B = -x / (x ** 2 + r ** 2)

        self.B_line = b / 2

        self.status = status

    def get_connections(self):
        return [(self.from_bus, self.to_bus)]

