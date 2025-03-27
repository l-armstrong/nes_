from __future__ import annotations
from bus import Bus
from dataclasses import dataclass
from fixed_types import uint16, uint8, check_type
from enum import IntEnum
from typing import Callable, List

@dataclass
class Instruction:
    name:       str
    operate:    Callable[[], uint8]
    addrmode:   Callable[[], uint8]
    cycles:     uint8

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
        # check_type(addr, uint16)
        return self.bus.read(addr, False)

    def write(self, addr: uint16, val: uint8) -> None:
        # write to bus
        # check_type(val, uint8)
        self.bus.write(addr, val)
    
    def get_flag(self, f: FLAGS) -> uint8:
        # get status of specfic bit
        return 1 if self.status & f > 0 else 0

    def set_flag(self, f: FLAGS, value: bool) -> None:
        # set bits in status register depending on flag
        if value: self.status |= f 
        else: self.status &= ~f

    # external signals
    def clock(self) -> None: 
        # one clock cycle to occur
        if (self.cycles == 0):
            # 1 byte opcode to index table
            self.opcode: uint8 = self.read(self.pc)
            print(f"{hex(self.pc)} {hex(self.opcode)} {self.lookup[self.opcode].name}\tA:{hex(self.a)} X:{hex(self.x)} Y:{hex(self.y)} SP:{hex(self.stkp)}")
            # increase pc after reading
            self.pc += 1
            # get current number of cycles required
            self.cycles = self.lookup[self.opcode].cycles
            # call function required for the opcodes address mode
            more_cycles_1 = self.lookup[self.opcode].addrmode()
            # call function required for the operate address mode
            more_cycles_2 = self.lookup[self.opcode].operate()
            # more_cycles_1 and more_cycles_2 return 1 or 0 if there needs to be another clock cycle
            self.cycles += (more_cycles_1 & more_cycles_2)
        # cycle is decremented every time the clock is called
        self.cycles -= 1
    # reset, irq, nmi can occur at anytime
    # need to behave async.
    # interrupts processor from doing what it is currently doing. 
    # only finishing current instruction
    def reset(self) -> None: 
        # when reset is called it configures the cpu into a known state
        self.a = 0
        self.x = 0
        self.y = 0
        self.stkp = 0xFD
        self.status = 0x00 | FLAGS.U
        # read 16 bit address for the program counter
        self.addr_abs: uint16 = uint16(0xFFFC)
        lo: uint16 = self.read(self.addr_abs + 0)
        hi: uint16 = self.read(self.addr_abs + 1)
        self.pc: uint16 = (hi << 8) | (lo)
        # reset result of internal variables
        self.addr_rel = 0x0000
        self.addr_abs = 0x0000
        self.fetched = 0x00
        self.cycles = 8
    def irq(self) -> None:
        # interrupt request signal
        # can be ignored depending if the interrupt enable flag is set
        if self.get_flag(FLAGS.I) == 0x00:
            # service interrupt
            # write program counter to the stack starting with hi byte
            self.write(0x0100 + self.stkp, (self.pc >> 8) & 0x00FF)
            self.stkp -= 1
            # then lo byte
            self.write(0x0100 + self.stkp, self.pc & 0x00FF)
            self.stkp -= 1
            # write status register
            self.set_flag(FLAGS.B, 0)
            self.set_flag(FLAGS.U, 1)
            self.set_flag(FLAGS.I, 1)
            self.write(0x0100 + self.stkp, self.status)
            self.stkp -= 1
            # get new program counter
            self.addr_abs = 0xFFFE
            lo: uint16 = self.read(self.addr_abs + 0)
            hi: uint16 = self.read(self.addr_abs + 1)
            self.pc = (hi << 8) | lo

            self.cycles = 7

    def nmi(self) -> None: 
        # non maskable interrupt request signal
        # service interrupt
        # write program counter to the stack starting with hi byte
        self.write(0x0100 + self.stkp, (self.pc >> 8) & 0x00FF)
        self.stkp -= 1
        # then lo byte
        self.write(0x0100 + self.stkp, self.pc & 0x00FF)
        self.stkp -= 1
        # write status register
        self.set_flag(FLAGS.B, 0)
        self.set_flag(FLAGS.U, 1)
        self.set_flag(FLAGS.I, 1)
        self.write(0x0100 + self.stkp, self.status)
        self.stkp -= 1
        # get new program counter
        self.addr_abs = 0xFFFA
        lo: uint16 = self.read(self.addr_abs + 0)
        hi: uint16 = self.read(self.addr_abs + 1)
        self.pc = (hi << 8) | lo
        self.cycles = 8

    def fetch(self) -> uint8:
        # fetch data from apporpriate source that does not 
        # check if current instr. has an address mode of implied.
        if self.lookup[self.opcode].addrmode is not self.imp:
            self.fetched = self.read(self.addr_abs)
        return uint8(self.fetched)
    
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
        self.addr_abs: uint16 = self.pc
        self.pc += 1
        return 0
    def zp0(self) -> uint8: 
        # zero page addressing
        self.addr_abs: uint16 = self.read(self.pc)
        self.pc += 1
        self.addr_abs &= 0x00FF
        return 0
    def zpx(self) -> uint8: 
        # zero page addressing w/ x register offset
        # useful for iterating through regions of memory like an array 
        self.addr_abs: uint16 = self.read(self.pc) + self.x
        self.pc += 1
        self.addr_abs &= 0x00FF
        return 0
    def zpy(self) -> uint8:
        # zero page addressing w/ y register offset
        # useful for iterating through regions of memory like an array 
        self.addr_abs: uint16 = self.read(self.pc) + self.y
        self.pc += 1
        self.addr_abs &= 0x00FF
        return 0
    def rel(self) -> uint8:
        # relative address mode, applies to branching instructions
        # can only jump to locations within a range of 127 memory location
        self.addr_rel: uint16 = self.read(self.pc)
        self.pc += 1
        # check if this is a signed bit, set hi byte to all ones
        if (self.addr_rel & 0x80): 
            # self.addr_rel |= 0xFF00
            self.addr_rel = self.addr_rel - 256 if self.addr_rel > 127 else self.addr_rel
        return 0
    def abs(self) -> uint8: 
        # specifies full address (3 byte instruction)
        lo = self.read(self.pc)
        self.pc += 1
        hi = self.read(self.pc)
        self.pc += 1
        self.addr_abs: uint16 = (hi << 8) | lo
        return 0
    def abx(self) -> uint8: 
        # specifies full address (3 byte instruction) x reg offset
        lo = self.read(self.pc)
        self.pc += 1
        hi = self.read(self.pc)
        self.pc += 1
        self.addr_abs: uint16 = (hi << 8) | lo
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
        self.addr_abs: uint16 = (hi << 8) | lo 
        self.addr_abs += self.y
        # if after inc by reg y we turn to a different page
        # we need to indicate to the system we need an additional
        # clock cycle
        # check if the hi byte has changed checking the hi byte are the same
        return 1 if (self.addr_abs & 0xFF00) != (hi << 8) else 0
    def ind(self) -> uint8:
        # 6502 way of implementing pointers
        lo = self.read(self.pc)
        self.pc += 1
        hi = self.read(self.pc)
        self.pc += 1
        # construct address
        ptr = uint16((hi << 8) | lo)
        # simulate page bounday hardware bug
        if lo == 0x00FF: self.addr_abs: uint16 = (self.read(ptr & 0xFF00) << 8) | self.read(ptr + 0)
        else: self.addr_abs: uint16 = (self.read(ptr + 1) << 8) | self.read(ptr + 0)
        return 0
    def izx(self) -> uint8: 
        # indirect addressing of the zero page with x reg offset
        # the address is somewhere in the zero page
        t = self.read(self.pc)
        self.pc += 1
        lo = self.read(uint16((t + self.x) & 0x00FF))
        hi = self.read(uint16((t + self.x + 1) & 0x00FF))
        self.addr_abs = (hi << 8) | lo
        return 0
    def izy(self) -> uint8:
        # indirect addressing of the zero page with y reg offset
        # the address is somewhere in the zero page
        # this is the actual address
        t = self.read(self.pc)
        self.pc += 1
        lo = self.read(uint16(t & 0x00FF))
        hi = self.read(uint16((t + 1) & 0xFF00))
        self.addr_abs = (hi << 8) | lo
        self.addr_abs += self.y
        # check if we changed the page boundary
        return 1 if (self.addr_abs & 0xFF00) != (hi << 8) else 0

    # Opcodes
    def adc(self) -> uint8:
        # add with carry
        self.fetch()
        temp: uint16 = uint16(self.a) + uint16(self.fetched) + uint16(self.get_flag(FLAGS.C))
        # check if carry
        self.set_flag(FLAGS.C, temp > 255)
        # check if result is zero
        self.set_flag(FLAGS.Z, (temp & 0x00FF == 0))
        # check if negative
        self.set_flag(FLAGS.N, temp & 0x80)
        # check if overflow
        self.set_flag(FLAGS.V, (~(uint16(self.a) ^ uint16(self.fetched))) & (uint16(self.a) ^ uint16(temp)) & 0x0080)
        #self.set_flag(FLAGS.V, (~(self.a ^ self.fetched) and (self.a ^ temp)) & 0x0080)
        # store the result
        self.a = temp & 0x00FF
        # could require an additional cycle
        return 1 
    def _and(self) -> uint8: 
        # perform bitwise & on a reg and fetched data
        # fetch data
        self.fetch()
        # perform bitwise
        self.a = uint8(self.a & self.fetched)
        # update status flags
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        # requires additional clock cycles
        return 1
    def asl(self) -> uint8: 
        # arithmetic shift left
        # fetch data
        self.fetch()
        tmp = uint16(self.fetched << 1)
        # update status flags
        self.set_flag(FLAGS.C, (tmp & 0xFF00) > 0)
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x00)
        self.set_flag(FLAGS.N, tmp & 0x80)
        # check if imp to handle case where we store in a reg
        if self.lookup[self.opcode].addrmode == self.imp:
            print("IN A")
            self.a = uint8(tmp)
        else:
            print("IN WRITE")
            self.write(self.addr_abs, uint8(tmp))
        
        return 0
    def bcc(self) -> uint8: 
        # branch if carry bit is clear
        if self.get_flag(FLAGS.C) == 0x00:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        for i, flag in enumerate(FLAGS):
            print(flag)
            print(self.status & flag)
        print("PC: ", hex(self.pc))
        if hex(self.pc) == "0xf939": exit(1)
        return 0
    def bcs(self) -> uint8: 
        # branch if carry bit is set
        if self.get_flag(FLAGS.C) == 0x01:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        return 0
    def beq(self) -> uint8:
        # branch if equal
        if self.get_flag(FLAGS.Z) == 0x01:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        return 0
    def bit(self) -> uint8: 
        # modifies flags, but does not change memory or registers
        self.fetch()
        # TODO: should cast both to uint16?
        tmp = uint16(self.a & self.fetched)
        # The zero flag is set depending on the result of the accumulator AND memory value
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x00)
        self.set_flag(FLAGS.N, self.fetched & (1 << 7))
        self.set_flag(FLAGS.V, self.fetched & (1 << 6))
        return 0
    def bmi(self) -> uint8: 
        # branch if negative
        if self.get_flag(FLAGS.N) == 0x01:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        return 0
    def bne(self) -> uint8: 
        # branch if not equal
        if self.get_flag(FLAGS.Z) == 0x00:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        for i, flag in enumerate(FLAGS):
            print(flag)
            print(self.status & flag)
        print("PC: ", hex(self.pc))
        if hex(self.pc) == "0xcf1c": exit(1)
        return 0
    def bpl(self) -> uint8: 
        # branch if positive
        if self.get_flag(FLAGS.N) == 0x00:
            self.cycles += 1
            self.addr_abs = uint16(self.pc + self.addr_rel)
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (uint16(self.pc) & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        return 0
    def brk(self) -> uint8:
        # break (software irq)
        # brk is a 2 byte instruction, however, we dont care about about second byte
        # this increment the pc by 1 before pushing on stack
        # push current program counter on stack
        self.pc += 1
        # set interrupt disable flag
        self.set_flag(FLAGS.I, True)
        self.write(0x0100 + self.stkp, (self.pc >> 8) & 0x00FF)
        self.stkp -= 1
        self.write(0x0100 + self.stkp, self.pc & 0xFF)
        self.stkp -= 1

        # set break flag
        self.set_flag(FLAGS.B, True)
        # push status reg on stack
        self.write(0x0100 + self.stkp, self.status)
        self.stkp -= 1
        # clear break flag
        self.set_flag(FLAGS.B, False)

        self.pc = uint16(self.read(0xFFFE) | (self.read(0xFFFF) << 8))
        return 0
        
    def bvc(self) -> uint8: 
        # branch if overflow is clear
        if self.get_flag(FLAGS.V) == 0x00:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        return 0
    def bvs(self) -> uint8:
        # branch if overflow
        # TODO: remove 0xc972 | 0xc784
        if self.get_flag(FLAGS.V) == 0x01:
            self.cycles += 1
            self.addr_abs = self.pc + self.addr_rel
            # check if we are crossing a page boundary
            if (self.addr_abs & 0xFF00) != (self.pc & 0xFF00):
                self.cycles += 1
            self.pc = self.addr_abs
        print("V flag:", type(self.get_flag(FLAGS.V)))
        for i, flag in enumerate(FLAGS):
            print(flag)
            print(self.status & flag)
        if hex(self.pc) == '0xc977': exit(1) # TODO: remove
        return 0
    def clc(self) -> uint8:
        # clear carry bit
        self.set_flag(FLAGS.C, False)
        return 0
    def cld(self) -> uint8:
        # clear decimal mode
        self.set_flag(FLAGS.D, False)
        return 0
    def cli(self) -> uint8: ...
    def clv(self) -> uint8: 
        # clears the overflow flag
        self.set_flag(FLAGS.V, False)
        return 0
    def cmp(self) -> uint8: 
        # compares A to a memory value
        # comparison is implemented as a subtraction
        self.fetch()
        tmp = uint16(self.a) - uint16(self.fetched)
        self.set_flag(FLAGS.C, self.a >= self.fetched)
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x0000)
        self.set_flag(FLAGS.N, tmp & 0x0080)
        return 1
    def cpx(self) -> uint8:
        # compares X to a memory value
        # comparison is implemented as a subtraction
        self.fetch()
        tmp = uint16(self.x) - uint16(self.fetched)
        self.set_flag(FLAGS.C, self.x >= self.fetched)
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x0000)
        self.set_flag(FLAGS.N, tmp & 0x0080)
        return 0

    def cpy(self) -> uint8: 
        # compares Y to a memory value
        # comparison is implemented as a subtraction
        self.fetch()
        tmp = uint16(self.y) - uint16(self.fetched)
        self.set_flag(FLAGS.C, self.y >= self.fetched)
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x0000)
        self.set_flag(FLAGS.N, tmp & 0x0080)
        return 0
    def dec(self) -> uint8: ...
    def dex(self) -> uint8: 
        # subtracts 1 from the X register.
        self.x = uint8(self.x - 1)
        self.set_flag(FLAGS.Z, self.x == 0x00)
        self.set_flag(FLAGS.N, self.x & 0x80)
        return 0
    def dey(self) -> uint8: 
        # decrement 1 from Y reg. Does not effect carry or overflow
        # decrement Y reg
        self.y = uint8(self.y - 1)
        # update flags
        self.set_flag(FLAGS.Z, self.y == 0x00)
        self.set_flag(FLAGS.N, self.y & 0x80)
        return 0
    def eor(self) -> uint8:
        # exclusive-ORs a memory value and the accumulator
        self.fetch()
        self.a = uint8(self.a ^ self.fetched)
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        return 1 
    def inc(self) -> uint8: ...
    def inx(self) -> uint8: 
        # INX adds 1 to the X register.
        self.x = uint8(self.x + 1)
        self.set_flag(FLAGS.Z, self.x == 0x00)
        self.set_flag(FLAGS.N, self.x & 0x80)
        return 0 
    def iny(self) -> uint8: 
        # INY adds 1 to the Y register
        self.y = uint8(self.y + 1)
        self.set_flag(FLAGS.Z, self.y == 0x00)
        self.set_flag(FLAGS.N, self.y & 0x80)
        return 0
    def jmp(self) -> uint8: 
        # sets the program counter to a new value, allowing code to execute from a new location.
        self.pc = self.addr_abs
        return 0
    
    def jsr(self) -> uint8: 
        # pushes the current program counter to the stack and then sets the program counter to a new value
        #  the return address on the stack points 1 byte before the start of the next instruction
        self.pc -= 1
        # push hi byte on stack
        self.write(0x0100 + self.stkp, (self.pc >> 8) & 0x00FF)
        self.stkp -= 1
        # push lo byte on stack
        self.write(0x0100 + self.stkp, self.pc & 0x00FF)
        self.stkp -= 1
        # go to new address
        self.pc = self.addr_abs
        return 0
    
    def lda(self) -> uint8: 
        # load a memory value into the A reg
        # fetch data to be read into fetched
        self.fetch()
        # load fetched data into X register
        self.a = uint8(self.fetched)
        # set flags
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        return 1
    def ldx(self) -> uint8: 
        # loads a memory value into the X register
        # fetch data to be read into fetched
        self.fetch()
        # load fetched data into X register
        self.x = uint8(self.fetched)
        # set flags
        self.set_flag(FLAGS.Z, self.x == 0x00)
        self.set_flag(FLAGS.N, self.x & 0x80)
        return 1
    def ldy(self) -> uint8: 
        # loads a memory value into the Y register
        # fetch data to be read into fetched
        self.fetch()
        # load fetched data into X register
        self.y = uint8(self.fetched)
        # set flags
        self.set_flag(FLAGS.Z, self.y == 0x00)
        self.set_flag(FLAGS.N, self.y & 0x80)
        return 1
    def lsr(self) -> uint8: 
        # shift all bits one pos to the right 
        self.fetch()
        # bit 0 is shifted into the carry flag
        self.set_flag(FLAGS.C, self.fetched & 0x0001)
        tmp = uint16(self.fetched >> 1)
        # set Zero Flag
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x0000)
        # set N Flag
        self.set_flag(FLAGS.N, tmp & 0x0080)
        print("DEBUG address mode:", self.lookup[self.opcode].addrmode is self.imp)
        # if self.lookup[self.opcode].addrmode is self.imp:
        if self.lookup[self.opcode].addrmode == self.imp:
            print("IN A")
            self.a = uint8(tmp & 0x00FF)
        else:
            print("IN WRITE")
            self.write(self.addr_abs, tmp & 0x00FF)
        return 0
        
    def nop(self) -> uint8:
        # has no effect. wastest space and CPU cycles. C
        # can be useful when writing timed code to delay for a desired amount of time
        match self.opcode:
            case 0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC: return 1
            case _: return 0 
    def ora(self) -> uint8: 
        # bitwise OR a memory value and the A reg.
        # fetch data
        self.fetch()
        print("SELF.A IN ORA:", self.a)
        self.a = uint8(self.a | self.fetched)
        print("self.fetched IN ORA:", self.fetched)
        # update status reg
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        return 1

    def pha(self) -> uint8: 
        # push accumulator to the stack
        # 0x0100 base location for stack pointer offset
        self.write(0x0100 + self.stkp, self.a)
        self.stkp -= 1
        return 0
    def php(self) -> uint8: 
        # stores a byte to the stack containing the 6 status flags and B flag and then decrements the stack pointer
        # push status flags on stack
        self.write(0x0100 + self.stkp, self.status | FLAGS.B | FLAGS.U)
        self.set_flag(FLAGS.B, False)
        self.set_flag(FLAGS.U, False)
        # dec sp
        self.stkp -= 1
        return 0
    def pla(self) -> uint8:
        # pop off the stack into accum
        self.stkp += 1
        # read from bus value we need
        self.a = uint8(self.read(0x0100 + self.stkp))
        # update flags
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        return 0
    def plp(self) -> uint8:
        # increments the stack pointer and then loads the value at that stack position into the 6 status flags.
        # pull value off stack into status register
        self.stkp = self.stkp + 1
        self.status = self.read(0x0100 + self.stkp)
        self.set_flag(FLAGS.U, True)
        return 0
    def rol(self) -> uint8:
        # shift memory value/acc to the left
        self.fetch()
        # the value in carry is shifted into bit 0
        tmp = uint16((uint16(self.fetched) << 1) | self.get_flag(FLAGS.C))
        # update flags
        self.set_flag(FLAGS.C, tmp & 0xFF00)
        self.set_flag(FLAGS.Z, (tmp & 0x00FF) == 0x0000)
        self.set_flag(FLAGS.N, tmp & 0x0080)
        if self.lookup[self.opcode].addrmode is self.imp: self.a = tmp & 0x00FF
        else: self.write(self.addr_abs, tmp & 0x00FF)
        return 0

    def ror(self) -> uint8: ...
    def rti(self) -> uint8: 
        # return from interrupt
        self.stkp += 1
        # restore status
        self.status = self.read(0x0100 + self.stkp)
        self.status &= ~FLAGS.B
        self.status &= ~FLAGS.U
        # restore pc
        self.stkp += 1
        # starting with lo byte
        self.pc = uint16(self.read(0x0100 + self.stkp))
        self.stkp += 1
        # then hi byte
        self.pc |= uint16(self.read(0x0100 + self.stkp) << 8)
        return 0
    def rts(self) -> uint8: 
        # pulls an address from the stack into the program counter and then increments the program counter
        self.stkp += 1
        # pull lo byte
        self.pc = uint16(self.read(0x0100 + self.stkp))
        self.stkp += 1
        # pull hi byte
        self.pc |= uint16(self.read(0x0100 + self.stkp) << 8)
        # go to next instruction
        self.pc += 1
        return 0
    def sbc(self) -> uint8:
        # subtract with carry
        self.fetch()
        # invert the bits of the data
        value: uint16 = uint16(self.fetched) ^ 0x00FF

        temp: uint16 = uint16(self.a) + uint16(value) + uint16(self.get_flag(FLAGS.C))
        # check if carry
        self.set_flag(FLAGS.C, temp & 0xFF00)
        # check if result is zero
        self.set_flag(FLAGS.Z, (temp & 0x00FF) == 0)
        # check if negative
        self.set_flag(FLAGS.N, temp & 0x0080)
        # check if overflow
        # SetFlag(V, (temp ^ (uint16_t)a) & (temp ^ value) & 0x0080)
        self.set_flag(FLAGS.V, (temp ^ uint16(self.a)) & (temp ^ value) & 0x0080)
        # store the result
        self.a = temp & 0x00FF
        return 1

    def sec(self) -> uint8: 
        # sets the carry flag. 
        # this is usually done before subtracting the low byte of a value with SBC 
        # to avoid subtracting an extra 1.
        self.set_flag(FLAGS.C, True)
        return 0

    def sed(self) -> uint8: 
        # sets the decimal flag
        self.set_flag(FLAGS.D, True)
        return 0
    def sei(self) -> uint8: 
        # SEI sets the interrupt disable flag, preventing the CPU from handling hardware IRQs
        self.set_flag(FLAGS.I, True)
        return 0
    def sta(self) -> uint8:
        # stores the accumulator value into memory
        self.write(self.addr_abs, self.a)
        return 0
    def stx(self) -> uint8: 
        # Stores the X reg value into memory
        # write
        self.write(self.addr_abs, self.x)
        return 0
    def sty(self) -> uint8: ...
    def tax(self) -> uint8: 
        # copies the accumulator value to the X register
        self.x = uint8(self.a)
        self.set_flag(FLAGS.Z, self.x == 0x00)
        self.set_flag(FLAGS.N, self.x & 0x80)
        return 0
    def tay(self) -> uint8: 
        # copies the accumulator value to the Y register
        self.y = uint8(self.a)
        self.set_flag(FLAGS.Z, self.y == 0x00)
        self.set_flag(FLAGS.N, self.y & 0x80)
        return 0
    def tsx(self) -> uint8: 
        # copies the stack pointer value to the X register.
        self.x = uint8(self.stkp)
        self.set_flag(FLAGS.Z, self.x == 0x00)
        self.set_flag(FLAGS.N, self.x & 0x80)
        return 0
    def txa(self) -> uint8: 
        # copies the X register value to the accumulator
        self.a = uint8(self.x)
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        return 0
    def txs(self) -> uint8: 
        # TXS copies the X register value to the stack pointer.
        self.stkp = self.x
        return 0
    def tya(self) -> uint8: 
        # copies the Y register value to the accumulator
        self.a = uint8(self.y)
        self.set_flag(FLAGS.Z, self.a == 0x00)
        self.set_flag(FLAGS.N, self.a & 0x80)
        return 0

    def xxx(self) -> uint8: return 0 # illegal opcode

    def complete(self) -> bool: return self.cycles == 0
    
    def generate_table(self) -> List[Instruction]:
        instruction_list: List[Instruction] = [
            Instruction("BRK", self.brk, self.imm, 7), Instruction("ORA", self.ora,  self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 3), Instruction("ORA", self.ora,  self.zp0, 3), Instruction("ASL", self.asl, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PHP", self.php, self.imp, 3), Instruction("ORA", self.ora,  self.imm, 2), Instruction("ASL", self.asl, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.nop, self.imp, 4), Instruction("ORA", self.ora, self.abs, 4),  Instruction("ASL", self.asl, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BPL", self.bpl, self.rel, 2), Instruction("ORA", self.ora,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("ORA", self.ora,  self.zpx, 4), Instruction("ASL", self.asl, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("CLC", self.clc, self.imp, 2), Instruction("ORA", self.ora,  self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("ORA", self.ora, self.abx, 4),  Instruction("ASL", self.asl, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("JSR", self.jsr, self.abs, 6), Instruction("AND", self._and, self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("BIT", self.bit, self.zp0, 3), Instruction("AND", self._and, self.zp0, 3), Instruction("ROL", self.rol, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PLP", self.plp, self.imp, 4), Instruction("AND", self._and, self.imm, 2), Instruction("ROL", self.rol, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("BIT", self.bit, self.abs, 4), Instruction("AND", self._and, self.abs, 4), Instruction("ROL", self.rol, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BMI", self.bmi, self.rel, 2), Instruction("AND", self._and, self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("AND", self._and, self.zpx, 4), Instruction("ROL", self.rol, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("SEC", self.sec, self.imp, 2), Instruction("AND", self._and, self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("AND", self._and, self.abx, 4), Instruction("ROL", self.rol, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("RTI", self.rti, self.imp, 6), Instruction("EOR", self.eor,  self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 3), Instruction("EOR", self.eor,  self.zp0, 3), Instruction("LSR", self.lsr, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PHA", self.pha, self.imp, 3), Instruction("EOR", self.eor,  self.imm, 2), Instruction("LSR", self.lsr, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("JMP", self.jmp, self.abs, 3), Instruction("EOR", self.eor, self.abs, 4),  Instruction("LSR", self.lsr, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BVC", self.bvc, self.rel, 2), Instruction("EOR", self.eor,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("EOR", self.eor,  self.zpx, 4), Instruction("LSR", self.lsr, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("CLI", self.cli, self.imp, 2), Instruction("EOR", self.eor,  self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("EOR", self.eor, self.abx, 4),  Instruction("LSR", self.lsr, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("RTS", self.rts, self.imp, 6), Instruction("ADC", self.adc,  self.izx, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 3), Instruction("ADC", self.adc,  self.zp0, 3), Instruction("ROR", self.ror, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("PLA", self.pla, self.imp, 4), Instruction("ADC", self.adc,  self.imm, 2), Instruction("ROR", self.ror, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("JMP", self.jmp, self.ind, 5), Instruction("ADC", self.adc, self.abs, 4),  Instruction("ROR", self.ror, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BVS", self.bvs, self.rel, 2), Instruction("ADC", self.adc,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("ADC", self.adc,  self.zpx, 4), Instruction("ROR", self.ror, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("SEI", self.sei, self.imp, 2), Instruction("ADC", self.adc,  self.aby, 4), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("ADC", self.adc, self.abx, 4),  Instruction("ROR", self.ror, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("???", self.nop, self.imp, 2), Instruction("STA", self.sta,  self.izx, 6), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 6), Instruction("STY", self.sty, self.zp0, 3), Instruction("STA", self.sta,  self.zp0, 3), Instruction("STX", self.stx, self.zp0, 3), Instruction("???", self.xxx, self.imp, 3), Instruction("DEY", self.dey, self.imp, 2), Instruction("???", self.nop,  self.imp, 2), Instruction("TXA", self.txa, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("STY", self.sty, self.abs, 4), Instruction("STA", self.sta, self.abs, 4),  Instruction("STX", self.stx, self.abs, 4), Instruction("???", self.xxx, self.imp, 4),
            Instruction("BCC", self.bcc, self.rel, 2), Instruction("STA", self.sta,  self.izy, 6), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 6), Instruction("STY", self.sty, self.zpx, 4), Instruction("STA", self.sta,  self.zpx, 4), Instruction("STX", self.stx, self.zpy, 4), Instruction("???", self.xxx, self.imp, 4), Instruction("TYA", self.tya, self.imp, 2), Instruction("STA", self.sta,  self.aby, 5), Instruction("TXS", self.txs, self.imp, 2), Instruction("???", self.xxx, self.imp, 5), Instruction("???", self.nop, self.imp, 5), Instruction("STA", self.sta, self.abx, 5),  Instruction("???", self.xxx, self.imp, 5), Instruction("???", self.xxx, self.imp, 5),
            Instruction("LDY", self.ldy, self.imm, 2), Instruction("LDA", self.lda,  self.izx, 6), Instruction("LDX", self.ldx, self.imm, 2), Instruction("???", self.xxx, self.imp, 6), Instruction("LDY", self.ldy, self.zp0, 3), Instruction("LDA", self.lda,  self.zp0, 3), Instruction("LDX", self.ldx, self.zp0, 3), Instruction("???", self.xxx, self.imp, 3), Instruction("TAY", self.tay, self.imp, 2), Instruction("LDA", self.lda,  self.imm, 2), Instruction("TAX", self.tax, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("LDY", self.ldy, self.abs, 4), Instruction("LDA", self.lda, self.abs, 4),  Instruction("LDX", self.ldx, self.abs, 4), Instruction("???", self.xxx, self.imp, 4),
            Instruction("BCS", self.bcs, self.rel, 2), Instruction("LDA", self.lda,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 5), Instruction("LDY", self.ldy, self.zpx, 4), Instruction("LDA", self.lda,  self.zpx, 4), Instruction("LDX", self.ldx, self.zpy, 4), Instruction("???", self.xxx, self.imp, 4), Instruction("CLV", self.clv, self.imp, 2), Instruction("LDA", self.lda,  self.aby, 4), Instruction("TSX", self.tsx, self.imp, 2), Instruction("???", self.xxx, self.imp, 4), Instruction("LDY", self.ldy, self.abx, 4), Instruction("LDA", self.lda, self.abx, 4),  Instruction("LDX", self.ldx, self.aby, 4), Instruction("???", self.xxx, self.imp, 4),
            Instruction("CPY", self.cpy, self.imm, 2), Instruction("CMP", self.cmp,  self.izx, 6), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("CPY", self.cpy, self.zp0, 3), Instruction("CMP", self.cmp,  self.zp0, 3), Instruction("DEC", self.dec, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("INY", self.iny, self.imp, 2), Instruction("CMP", self.cmp,  self.imm, 2), Instruction("DEX", self.dex, self.imp, 2), Instruction("???", self.xxx, self.imp, 2), Instruction("CPY", self.cpy, self.abs, 4), Instruction("CMP", self.cmp, self.abs, 4),  Instruction("DEC", self.dec, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BNE", self.bne, self.rel, 2), Instruction("CMP", self.cmp,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("CMP", self.cmp,  self.zpx, 4), Instruction("DEC", self.dec, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("CLD", self.cld, self.imp, 2), Instruction("CMP", self.cmp,  self.aby, 4), Instruction("NOP", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("CMP", self.cmp, self.abx, 4),  Instruction("DEC", self.dec, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
            Instruction("CPX", self.cpx, self.imm, 2), Instruction("SBC", self.sbc,  self.izx, 6), Instruction("???", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("CPX", self.cpx, self.zp0, 3), Instruction("SBC", self.sbc,  self.zp0, 3), Instruction("INC", self.inc, self.zp0, 5), Instruction("???", self.xxx, self.imp, 5), Instruction("INX", self.inx, self.imp, 2), Instruction("SBC", self.sbc,  self.imm, 2), Instruction("NOP", self.nop, self.imp, 2), Instruction("???", self.sbc, self.imp, 2), Instruction("CPX", self.cpx, self.abs, 4), Instruction("SBC", self.sbc, self.abs, 4),  Instruction("INC", self.inc, self.abs, 6), Instruction("???", self.xxx, self.imp, 6),
            Instruction("BEQ", self.beq, self.rel, 2), Instruction("SBC", self.sbc,  self.izy, 5), Instruction("???", self.xxx, self.imp, 2), Instruction("???", self.xxx, self.imp, 8), Instruction("???", self.nop, self.imp, 4), Instruction("SBC", self.sbc,  self.zpx, 4), Instruction("INC", self.inc, self.zpx, 6), Instruction("???", self.xxx, self.imp, 6), Instruction("SED", self.sed, self.imp, 2), Instruction("SBC", self.sbc,  self.aby, 4), Instruction("NOP", self.nop, self.imp, 2), Instruction("???", self.xxx, self.imp, 7), Instruction("???", self.nop, self.imp, 4), Instruction("SBC", self.sbc, self.abx, 4),  Instruction("INC", self.inc, self.abx, 7), Instruction("???", self.xxx, self.imp, 7),
        ]
        return instruction_list