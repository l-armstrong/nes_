from __future__ import annotations
from cpu_6502 import CPU_6502, FLAGS
from bus import Bus
from fixed_types import uint16, uint8

cpu = CPU_6502()
bus = Bus(cpu)

if __name__ == '__main__':
    program = list(map(lambda x: int(x, 16), "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA".split(' ')))
    index = 0x8000
    for c in program:
        bus.ram[index] = c
        index += 1
    bus.ram[0xFFFC] = 0x00
    bus.ram[0xFFFD] = 0x80

    cpu.reset()
    count = 0
    while True:
        count += 1
        cpu.clock()
        while not cpu.complete():
            cpu.clock()
        print(bus.ram[:20])

        if count == 20: break
