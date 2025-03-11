from __future__ import annotations
from cpu_6502 import CPU_6502
from bus import Bus
from fixed_types import uint16, uint8

cpu = CPU_6502()
bus = Bus(cpu)

x = uint16(0x0000)
print("x type:", type(x))
y = uint8(5)

x: uint16 = y + x
print("x type:", type(x))