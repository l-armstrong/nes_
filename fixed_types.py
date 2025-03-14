# def uint16(value: int) -> int:
#     return value & 0xFFFF

# def uint8(value: int) -> int:
#     return value & 0xFF

class uint16(int):
    def __new__(cls, val: int):
        val &= 0xFFFF   # mask 16 bit value
        return super().__new__(cls, val)
    def __add__(self, other):
        return uint16(int(self) + int(other))  # Preserve Uint16 type
    def __sub__(self, other):
        return uint16(int(self) - int(other))  # Preserve Uint16 type

class uint8(int):
    def __new__(cls, val: int):
        val &= 0xFF     # mask 8 bit value
        return super().__new__(cls, val)
    def __add__(self, other):
        return uint8(int(self) + int(other))  # Preserve Uint8 type
    def __sub__(self, other):
        return uint8(int(self) - int(other))  # Preserve Uint8 type

def check_type(val, _type):
    if not isinstance(val, _type):
        raise TypeError(f"Expected val: {val} for be type: {_type} not {type(val)}")
