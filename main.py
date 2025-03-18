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
    # with open("nestest.nes", "rb") as rom:
    #     index = 0x8000
    #     while byte := rom.read(1):
    #         bus.ram[index] = int.from_bytes(byte, byteorder='little')
    #         index += 1
    
    # bus.ram[0xFFFC] = 0x00
    # bus.ram[0xFFFD] = 0x80
    # with open("nestest.nes", "rb") as rom:
    #     index = 0xc000
    #     while byte := rom.read(1):
    #         bus.ram[index] = int.from_bytes(byte, byteorder='little')
    #         index += 1
    with open("nestest.nes", "rb") as f:
        rom = f.read()
    
    header = rom[:16]
    rom_size = header[4] * 0x4000
    program = rom[16:16+rom_size]
    size = len(program)
    if size == 0x4000:
        bus.ram[0x8000:0xC000] = program
        bus.ram[0xC000:0x10000] = program
    elif size == 0x8000:
        bus.ram[0x8000:0x10000]
    else:
        raise ValueError("Unsupported PRG ROM size")
    
    cpu.reset()
    cpu.pc = 0xc000
    while True:
        cpu.clock()
        #print(f"{hex(cpu.pc)} {hex(cpu.opcode)} {cpu.lookup[cpu.opcode].name}\tA:{hex(cpu.a)} X:{hex(cpu.x)} Y:{hex(cpu.y)} SP:{hex(cpu.stkp)}")
        while not cpu.complete():
            cpu.clock()
        print()
