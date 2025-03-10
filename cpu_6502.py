from __future__ import annotations
from bus import Bus
from fixed_types import uint16, uint8, check_type
from enum import IntEnum

class FLAGS(IntEnum):
    C = (1 << 0)    # Carry Bit
    Z = (1 << 1)    # Zero 
    I = (1 << 2)    # Disable Interrupts
    D = (1 << 3)    # Decimal Mode (unused in )
    B = (1 << 4)    # Break
    U = (1 << 5)    # Unused
    V = (1 << 6)    # Overflow
    N = (1 << 7)    # Negative

class CPU_6502(object):
    def __init__(self):
        pass

    def connect_bus(self, bus: Bus) -> None:
        check_type(bus, Bus)
        self.bus = bus
    
    def read(self, addr: uint16) -> uint8:
        check_type(addr, uint16)
        return self.bus.read(addr, False)

    def write(self, addr: uint16, val: uint8) -> None:
        check_type(val, uint8)
        self.bus.write(addr, val)