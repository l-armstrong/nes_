from __future__ import annotations
from cpu_6502 import CPU_6502
from bus import Bus

if __name__ == '__main__':
    cpu = CPU_6502()
    bus = Bus(cpu)
    # program = list(map(lambda x: int(x, 16), "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA".split(' ')))
    # index = 0x8000
    # for c in program:
    #     bus.ram[index] = c
    #     index += 1
    # bus.ram[0xFFFC] = 0x00
    # bus.ram[0xFFFD] = 0x80
    with open("nestest.nes", "rb") as rom:
        index = 0x8000
        while byte := rom.read(1):
            bus.ram[index] = int.from_bytes(byte, byteorder='little')
            index += 1
    
    bus.ram[0xFFFC] = 0x00
    bus.ram[0xFFFD] = 0x80

    cpu.reset()
    count = 0
    while True:
        count += 1
        cpu.clock()
        print("DEBUG:self.lookup[opcode]=", cpu.lookup[cpu.opcode])
        while not cpu.complete():
            cpu.clock()
        # if count == 43: break
        print()
