# def uint16(value: int) -> int:
#     return value & 0xFFFF

# def uint8(value: int) -> int:
#     return value & 0xFF

class uint16(int):
    def __new__(cls, val: int):
        val &= 0xFFFF   # mask 16 bit value
        return super().__new__(cls, val)

class uint8(int):
    def __new__(cls, val: int):
        val &= 0xFF     # mask 8 bit value
        return super().__new__(cls, val)

def check_type(val, _type):
    if not isinstance(val, _type):
        raise TypeError(f"Expected val: {val} for be type: {_type} not {type(val)}")
