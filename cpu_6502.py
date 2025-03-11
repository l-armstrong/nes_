from __future__ import annotations
from bus import Bus
from dataclasses import dataclass
from fixed_types import uint16, uint8, check_type
from enum import IntEnum
from typing import Callable, List

@dataclass
class Instruction:
    name:       str
    cycles:     uint8
    operate:    Callable[[], uint8]
    addrmode:   Callable[[], uint8]

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
        self.a = uint8(0x00)            # Accumulator register
        self.x = uint8(0x00)            # X register 
        self.y = uint8(0x00)            # Y register 
        self.stkp = uint16(0x00)        # Stack Pointer 
        self.pc = uint16(0x00)          # Program Counter
        self.status = uint8(0x00)       # status register

        self.fetched = uint8(0x00)      # current fetched data
        self.addr_abs = uint16(0x0000)  # can be used to read from diff location of memory.
        self.addr_rel = uint16(0x0000)  # used for branching, to jump a certain amount of distance
        self.opcode = uint8(0x00)       # current opcode
        self.cycles = uint8(0x00)       # stores the number of cycles left for the current instruction.
        self.lookup: List[Instruction] = self.generate_table()

    def connect_bus(self, bus: Bus) -> None:
        # connect cpu to bus
        check_type(bus, Bus)
        self.bus = bus
    
    def read(self, addr: uint16) -> uint8:
        # read from bus
        check_type(addr, uint16)
        return self.bus.read(addr, False)

    def write(self, addr: uint16, val: uint8) -> None:
        # write to bus
        check_type(val, uint8)
        self.bus.write(addr, val)
    
    def get_flag(f: FLAGS) -> uint8: ...

    def set_flag(f: FLAGS, value: bool) -> None:
        # set bits in status register depending on flag
        ...

    # external signals
    def clock(self) -> None: 
        # one clock cycle to occur
        if (self.cycles == 0):
            # 1 byte opcode to index table
            opcode: uint8 = self.read(self.pc)
            # increase pc after reading
            self.pc += 1
            # get current number of cycles required
            self.cycles = self.lookup[opcode].cycles
            # call function required for the opcodes address mode
            more_cycles_1 = self.lookup[opcode].addrmode()
            # call function required for the operate address mode
            more_cycles_2 = self.lookup[opcode].operate()
            # more_cycles_1 and more_cycles 2 return 1 or 0 if there needs to be another clock cycle
            self.cycles += (more_cycles_1 & more_cycles_2)
        # cycle is decremented every time the clock is called
        self.cycles -= 1
    # reset, irq, nmi can occur at anytime
    # need to behave async.
    # interrupts processor from doing what it is currently doing. 
    # only finishing current instruction
    def reset(self) -> None: ...
    def irq(self) -> None:
        # interrupt request signal
        # can be ignored depeninding if the interrupt enable flag is set
        ...
    def nmi(self) -> None: 
        # non maskable interrupt request signal
        ...

    def fetch(self) -> uint8:
        # fetch data from apporpriate source
        ...
    
    # Adressing Modes 
    def imp(self) -> uint8: 
        # could be operation on the accumulator register
        self.fetched = self.a
        # there are no data as part of instruction
        return 0
    def imm(self) -> uint8: 
        # the data is supplied as part of the instruction
        # its the next byte
        # all address mode set the addr abs, so the instruction know where to read data
        self.addr_abs = self.pc
        self.pc += 1
        return 0
    def zp0(self) -> uint8: 
        # zero page addressing
        self.addr_abs = self.read(self.pc)
        self.pc += 1
        self.addr_abs &= 0x00FF
        return 0
    def zpx(self) -> uint8: 
        # zero page addressing w/ x register offset
        # useful for iterating through regions of memory like an array 
        self.addr_abs = self.read(self.pc) + self.x
        self.pc += 1
        self.addr_abs &= 0x00FF
        return 0
    def zpy(self) -> uint8:
        # zero page addressing w/ y register offset
        # useful for iterating through regions of memory like an array 
        self.addr_abs = self.read(self.pc) + self.y
        self.pc += 1
        self.addr_abs &= 0x00FF
        return 0
    def rel(self) -> uint8: ...
    def abs(self) -> uint8: 
        # specifies full address (3 byte instruction)
        lo = self.read(self.pc)
        self.pc += 1
        hi = self.read(self.pc)
        self.pc += 1
        self.addr_abs = (hi << 8) | lo
        return 0
    def abx(self) -> uint8: 
        # specifies full address (3 byte instruction) x reg offset
        lo = self.read(self.pc)
        self.pc += 1
        hi = self.read(self.pc)
        self.pc += 1
        self.addr_abs = (hi << 8) | lo
        self.addr_abs += self.x
        # if after inc by reg x we turn to a different page
        # we need to indicate to the system we need an additional
        # clock cycle
        # check if the hi byte has changed checking the hi byte are the same
        return 1 if (self.addr_abs & 0xFF00) != (hi << 8) else 0
    
    def aby(self) -> uint8: 
        # specifies full address (3 byte instruction) y reg offset
        lo = self.read(self.pc)
        self.pc += 1
        hi = self.read(self.pc)
        self.pc += 1
        self.addr_abs = (hi << 8) | lo 
        self.addr_abs += self.y
        # if after inc by reg y we turn to a different page
        # we need to indicate to the system we need an additional
        # clock cycle
        # check if the hi byte has changed checking the hi byte are the same
        return 1 if (self.addr_abs & 0xFF00) != (hi << 8) else 0
    def ind(self) -> uint8: ...
    def izx(self) -> uint8: ...
    def izy(self) -> uint8: ...

    # Opcodes
    def adc(self) -> uint8: ...
    def _and(self) -> uint8: ...
    def asl(self) -> uint8: ...
    def bcc(self) -> uint8: ...
    def bcs(self) -> uint8: ...
    def beq(self) -> uint8: ...
    def bit(self) -> uint8: ...
    def bmi(self) -> uint8: ...
    def bne(self) -> uint8: ...
    def bpl(self) -> uint8: ...
    def brk(self) -> uint8: ...
    def bvc(self) -> uint8: ...
    def bvs(self) -> uint8: ...
    def clc(self) -> uint8: ...
    def cld(self) -> uint8: ...
    def cli(self) -> uint8: ...
    def clv(self) -> uint8: ...
    def cmp(self) -> uint8: ...
    def cpx(self) -> uint8: ...
    def cpy(self) -> uint8: ...
    def dec(self) -> uint8: ...
    def dex(self) -> uint8: ...
    def dey(self) -> uint8: ...
    def eor(self) -> uint8: ...
    def inc(self) -> uint8: ...
    def inx(self) -> uint8: ...
    def iny(self) -> uint8: ...
    def jmp(self) -> uint8: ...
    def jsr(self) -> uint8: ...
    def lda(self) -> uint8: ...
    def ldx(self) -> uint8: ...
    def ldy(self) -> uint8: ...
    def lsr(self) -> uint8: ...
    def nop(self) -> uint8: ...
    def ora(self) -> uint8: ...
    def pha(self) -> uint8: ...
    def php(self) -> uint8: ...
    def pla(self) -> uint8: ...
    def plp(self) -> uint8: ...
    def rol(self) -> uint8: ...
    def ror(self) -> uint8: ...
    def rti(self) -> uint8: ...
    def rts(self) -> uint8: ...
    def sbc(self) -> uint8: ...
    def sec(self) -> uint8: ...
    def sed(self) -> uint8: ...
    def sei(self) -> uint8: ...
    def sta(self) -> uint8: ...
    def stx(self) -> uint8: ...
    def sty(self) -> uint8: ...
    def tax(self) -> uint8: ...
    def tay(self) -> uint8: ...
    def tsx(self) -> uint8: ...
    def txa(self) -> uint8: ...
    def txs(self) -> uint8: ...
    def tya(self) -> uint8: ...

    def xxx(self) -> uint8: ... 
    
    def generate_table(self) -> List[Instruction]:
        instruction_list = List[Instruction] = [
            Instruction("BRK", self.brk, self.imm, 7), Instruction("ORA", self.ora,  self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 3), Instruction("ORA", self.ora,  self.zp0, 3), Instruction("ASL", self.asl, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PHP", self.php, self.imp, 3), Instruction("ORA", self.ora,  self.imp, 2), Instruction("ASL", self.asl, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.nop, self.imp, 4), Instruction("ORA", self.ora, self.abs, 4),  Instruction("ASL", self.asl, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BPL", self.bpl, self.rel, 2), Instruction("ORA", self.ora,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("ORA", self.ora,  self.zpx, 4), Instruction("ASL", self.asl, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("CLC", self.clc, self.imp, 2), Instruction("ORA", self.ora,  self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("ORA", self.ora, self.abx, 4),  Instruction("ASL", self.asl, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("JSR", self.jsr, self.abs, 6), Instruction("AND", self._and, self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("BIT", self.bit, self.zp0, 3), Instruction("AND", self._and, self.zp0, 3), Instruction("ROL", self.rol, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PLP", self.plp, self.imp, 4), Instruction("AND", self._and, self.imp, 2), Instruction("ROL", self.rol, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("BIT", self.bit, self.abs, 4), Instruction("AND", self._and, self.abs, 4), Instruction("ROL", self.rol, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BMI", self.bmi, self.rel, 2), Instruction("AND", self._and, self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("AND", self._and, self.zpx, 4), Instruction("ROL", self.rol, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("SEC", self.sec, self.imp, 2), Instruction("AND", self._and, self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("AND", self._and, self.abx, 4), Instruction("ROL", self.rol, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("RTI", self.rti, self.imp, 6), Instruction("EOR", self.eor,  self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 3), Instruction("EOR", self.eor,  self.zp0, 3), Instruction("LSR", self.lsr, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PHA", self.pha, self.imp, 3), Instruction("EOR", self.eor,  self.imp, 2), Instruction("LSR", self.lsr, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("JMP", self.jmp, self.abs, 3), Instruction("EOR", self.eor, self.abs, 4),  Instruction("LSR", self.lsr, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BVC", self.bvc, self.rel, 2), Instruction("EOR", self.eor,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("EOR", self.eor,  self.zpx, 4), Instruction("LSR", self.lsr, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("CLI", self.cli, self.imp, 2), Instruction("EOR", self.eor,  self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("EOR", self.eor, self.abx, 4),  Instruction("LSR", self.lsr, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("RTS", self.rts, self.imp, 6), Instruction("ADC", self.adc,  self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 3), Instruction("ADC", self.adc,  self.zp0, 3), Instruction("ROR", self.ror, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PLA", self.pla, self.imp, 4), Instruction("ADC", self.adc,  self.imp, 2), Instruction("ROR", self.ror, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("JMP", self.jmp, self.ind, 5), Instruction("ADC", self.adc, self.abs, 4),  Instruction("ROR", self.ror, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BVS", self.bvs, self.rel, 2), Instruction("ADC", self.adc,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("ADC", self.adc,  self.zpx, 4), Instruction("ROR", self.ror, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("SEI", self.sei, self.imp, 2), Instruction("ADC", self.adc,  self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("ADC", self.adc, self.abx, 4),  Instruction("ROR", self.ror, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("???", self.nop, self.imp, 2), Instruction("STA", self.sta,  self.izx, 6), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 6), Instruction("STY", self.sty, self.zp0, 3), Instruction("STA", self.sta,  self.zp0, 3), Instruction("STX", self.stx, self.zp0, 3), Instruction("???", self.xxx, self.imp, 3), Instruction("DEY", self.dey, self.imp, 2), Instruction("???", self.nop,  self.imp, 2), Instruction("TXA", self.txa, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("STY", self.sty, self.abs, 4), Instruction("STA", self.sta, self.abs, 4),  Instruction("STX", self.stx, self.abs, 4), Instruction("???", self.xxx, self.imp, 4),
            Instruction("BCC", self.bcc, self.rel, 2), Instruction("STA", self.sta,  self.izy, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 6), Instruction("STY", self.sty, self.zpx, 4), Instruction("STA", self.sta,  self.zpx, 4), Instruction("STX", self.stx, self.zpy, 4), Instruction("???", self.xxx, self.imp, 4), Instruction("TYA", self.tya, self.imp, 2), Instruction("STA", self.sta,  self.aby, 5), Instruction("TXS", self.txs, self.imp, 2), Instruction("???", self.xxx, self.imp, 5), Instruction("???", self.nop, self.imp, 5), Instruction("STA", self.sta, self.abx, 5),  Instruction("???", self.xxx, self.imp, 5), Instruction("???", self.xxx, self.imp, 5),
            Instruction("LDY", self.ldy, self.imp, 2), Instruction("LDA", self.lda,  self.izx, 6), Instruction("LDX", self.ldx, self.imp, 2), Instruction("???", self.xxx, self.imp, 6), Instruction("LDY", self.ldy, self.zp0, 3), Instruction("LDA", self.lda,  self.zp0, 3), Instruction("LDX", self.ldx, self.zp0, 3), Instruction("???", self.xxx, self.imp, 3), Instruction("TAY", self.tay, self.imp, 2), Instruction("LDA", self.lda,  self.imp, 2), Instruction("TAX", self.tax, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("LDY", self.ldy, self.abs, 4), Instruction("LDA", self.lda, self.abs, 4),  Instruction("LDX", self.ldx, self.abs, 4), Instruction("???", self.xxx, self.imp, 4),
            Instruction("BCS", self.bcs, self.rel, 2), Instruction("LDA", self.lda,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 5), Instruction("LDY", self.ldy, self.zpx, 4), Instruction("LDA", self.lda,  self.zpx, 4), Instruction("LDX", self.ldx, self.zpy, 4), Instruction("???", self.xxx, self.imp, 4), Instruction("CLV", self.clv, self.imp, 2), Instruction("LDA", self.lda,  self.aby, 4), Instruction("TSX", self.tsx, self.imp, 2), Instruction("???", self.xxx, self.imp, 4), Instruction("LDY", self.ldy, self.abx, 4), Instruction("LDA", self.lda, self.abx, 4),  Instruction("LDX", self.ldx, self.aby, 4), Instruction("???", self.xxx, self.imp, 4),
            Instruction("CPY", self.cpy, self.imp, 2), Instruction("CMP", self.cmp,  self.izx, 6), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("CPY", self.cpy, self.zp0, 3), Instruction("CMP", self.cmp,  self.zp0, 3), Instruction("DEC", self.dec, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("INY", self.iny, self.imp, 2), Instruction("CMP", self.cmp,  self.imp, 2), Instruction("DEX", self.dex, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("CPY", self.cpy, self.abs, 4), Instruction("CMP", self.cmp, self.abs, 4),  Instruction("DEC", self.dec, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BNE", self.bne, self.rel, 2), Instruction("CMP", self.cmp,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("CMP", self.cmp,  self.zpx, 4), Instruction("DEC", self.dec, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("CLD", self.cld, self.imp, 2), Instruction("CMP", self.cmp,  self.aby, 4), Instruction("NOP", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("CMP", self.cmp, self.abx, 4),  Instruction("DEC", self.dec, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("CPX", self.cpx, self.imp, 2), Instruction("SBC", self.sbc,  self.izx, 6), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("CPX", self.cpx, self.zp0, 3), Instruction("SBC", self.sbc,  self.zp0, 3), Instruction("INC", self.inc, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("INX", self.inx, self.imp, 2), Instruction("SBC", self.sbc,  self.imp, 2), Instruction("NOP", self.nop, self.imp, 2), Instruction("???", self.sbc, self.imp, 2), Instruction("CPX", self.cpx, self.abs, 4), Instruction("SBC", self.sbc, self.abs, 4),  Instruction("INC", self.inc, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BEQ", self.beq, self.rel, 2), Instruction("SBC", self.sbc,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("SBC", self.sbc,  self.zpx, 4), Instruction("INC", self.inc, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("SED", self.sed, self.imp, 2), Instruction("SBC", self.sbc,  self.aby, 4), Instruction("NOP", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("SBC", self.sbc, self.abx, 4),  Instruction("INC", self.inc, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
        ]
        return instruction_list