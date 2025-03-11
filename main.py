from __future__ import annotations
from cpu_6502 import CPU_6502, FLAGS
from bus import Bus
from fixed_types import uint16, uint8

cpu = CPU_6502()
bus = Bus(cpu)
