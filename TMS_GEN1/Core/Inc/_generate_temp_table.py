# Just run this file. Requires numpy.
# Output goes into temp_lookup_table.h >> temp_lookup_table

from __future__ import annotations

import logging
from itertools import starmap

import numpy as np

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel("INFO")


class Monad:
    def __init__(self, value: float):
        self._value = value

    def apply(self, function: callable) -> Monad:
        v_in = self.value
        v_out = function(self.value)
        logger.debug(f"{v_in} -> {function.__name__} -> {v_out}")
        return Monad(v_out)

    @property
    def value(self) -> float:
        return self._value


def temp_to_voltsensor(temp: float):
    table = np.array(
        [
            [-40, 2.44],
            [-35, 2.42],
            [-30, 2.40],
            [-25, 2.38],
            [-20, 2.35],
            [-15, 2.32],
            [-10, 2.27],
            [-5, 2.23],
            [0, 2.17],
            [5, 2.11],
            [10, 2.05],
            [15, 1.99],
            [20, 1.92],
            [25, 1.86],
            [30, 1.80],
            [35, 1.74],
            [40, 1.68],
            [45, 1.63],
            [50, 1.59],
            [55, 1.55],
            [60, 1.51],
            [65, 1.48],
            [70, 1.45],
            [75, 1.43],
            [80, 1.40],
            [85, 1.38],
            [90, 1.37],
            [95, 1.35],
            [100, 1.34],
            [105, 1.33],
            [110, 1.32],
            [115, 1.31],
            [120, 1.30],
        ]
    )
    return np.interp(temp, *table.T)


def voltsensor_to_voltadc(voltstm: float) -> float:
    volt_divided = 0.836 * voltstm
    return 1.44 + volt_divided / 2


def voltadc_to_adc(voltadc: float) -> int:
    return int(voltadc * 4095 / 3.3)


def temp_to_adc(temp: float) -> int:
    return (
        Monad(temp)
        .apply(temp_to_voltsensor)
        .apply(voltsensor_to_voltadc)
        .apply(voltadc_to_adc)
        .value
    )


def temp_to_fan(temp: float) -> int:
    # copied from original table
    return np.piecewise(
        temp,
        [temp < 0, (temp >= 0) & (temp < 50)],
        [0, 30 + 70 / 50 * temp, 100],
    )


def table_entry(temp):
    adc = temp_to_adc(temp)
    fan = temp_to_fan(temp)
    return f"{{{adc}, {temp}, {fan}}},"


temperatures = range(-40, 120 + 1, 5)
entries = map(table_entry, temperatures)
print("\n".join(entries))
