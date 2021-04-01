from functools import reduce
import operator as op
import numpy as np


class Hamming:
    def __init__(self, block_size: int, include_0_bit=False) -> None:
        self.block_size = block_size
        self.include_0_bit = include_0_bit

        if not self.include_0_bit:
            # the code will still calculate the 0 bit but won't show it to the user
            self.block_size += 1

        self.message_len = self.block_size - \
            int(np.ceil(np.log2(self.block_size))) - 1

    def _is_control_bit(self, num: int) -> bool:
        """
        :returns: whether the `num` is a power of `2` or `== 0`
        """
        assert num >= 0

        return (num & (num - 1) == 0)

    def _hamming_xor(self, block: np.array) -> int:
        """
        :returns: calculated checksum bits if the checksum is empty or wrong bit. 0 == no wrong bit is identified
        """
        assert len(block) == self.block_size

        return reduce(op.xor, (i for i, bit in enumerate(block) if bit), 0)

    def _extract_message(self, block: np.array) -> np.array:
        assert len(block) == self.block_size

        return np.array([bit for i, bit in enumerate(block) if not self._is_control_bit(i)], dtype=np.byte)

    def decode_block(self, encoded: np.array) -> np.array:
        """
        :encoded: the array of *bits*
        """
        if not self.include_0_bit:
            encoded = np.hstack([0, encoded])

        assert len(encoded) == self.block_size

        mistake_num = self._hamming_xor(encoded)
        if mistake_num:
            encoded[mistake_num] = not encoded[mistake_num]

        if self.include_0_bit:
            oddness = reduce(op.xor, encoded[1:], 0)
            if (oddness != encoded[0]):
                raise ValueError("Unable to decode")

        return self._extract_message(encoded)

    def encode_block(self, message: np.array) -> np.array:
        assert len(message) <= self.message_len

        ret = np.zeros(self.block_size, dtype=np.byte)
        check_bits = []
        i = 0
        for bit in message:
            while self._is_control_bit(i):
                check_bits.append(i)
                ret[i] = 0
                i += 1
            ret[i] = bit
            i += 1

        checksum = self._hamming_xor(ret)
        for bit_n in check_bits:
            ret[bit_n] = (checksum & bit_n) != 0

        ret[0] = reduce(op.xor, ret[1:], 0)

        if not self.include_0_bit:
            ret = ret[1:]

        return ret

