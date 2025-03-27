from __future__ import annotations
from fixed_types import uint16, uint8, check_type


class Bus(object):
    def __init__(self, cpu):
        self.cpu = cpu
        cpu.connect_bus(self)
        self.ram = [uint8(0) for _ in range(64 * 1024)] # 64 KB ram
    
    def write(self, addr: uint16, data: uint8) -> None:
        # check_type(data, uint8)
        # check_type(addr, uint16)
        # guard ram
        # TODO should be convert data to uint8?
        if (addr >= 0x0000 and addr <= 0xFFFF):
            self.ram[addr] = data
    
    def read(self, addr: uint16, b_read_only: bool = False) -> uint8:
        # check_type(addr, uint16)
        if (addr >= 0x0000 and addr <= 0xFFFF):
            return self.ram[addr]
        return 0x00