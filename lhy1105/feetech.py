"""Small, read-only FT-SCS transport for the FE-URT-2 TTL bus."""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial


class ServoError(RuntimeError):
    pass


@dataclass(frozen=True)
class ServoSample:
    positions: dict[int, int]
    completed_at: float


class FeetechBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.035):
        self.serial = serial.Serial(port, baudrate=baudrate, timeout=0.005, write_timeout=0.1)
        self.timeout = timeout

    def close(self) -> None:
        self.serial.close()

    def __enter__(self) -> "FeetechBus":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @staticmethod
    def packet(servo_id: int, instruction: int, parameters: bytes = b"") -> bytes:
        if not 0 <= servo_id <= 253:
            raise ValueError("servo ID out of range")
        body = bytes((servo_id, len(parameters) + 2, instruction)) + parameters
        return b"\xff\xff" + body + bytes(((~sum(body)) & 0xFF,))

    @staticmethod
    def parse_reply(raw: bytes, servo_id: int, parameter_count: int) -> bytes:
        if len(raw) != parameter_count + 6 or raw[:4] != bytes((255, 255, servo_id, parameter_count + 2)):
            raise ServoError(f"ID {servo_id}: invalid response {raw.hex(' ')}")
        if raw[-1] != ((~sum(raw[2:-1])) & 0xFF):
            raise ServoError(f"ID {servo_id}: checksum error")
        if raw[4] != 0:
            raise ServoError(f"ID {servo_id}: device error 0x{raw[4]:02x}")
        return raw[5:-1]

    def _exchange(self, servo_id: int, instruction: int, parameters: bytes, response_bytes: int) -> bytes:
        request = self.packet(servo_id, instruction, parameters)
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.serial.flush()
        deadline = time.monotonic() + self.timeout
        buffer = bytearray()
        expected = response_bytes + 6
        while time.monotonic() < deadline:
            chunk = self.serial.read(max(1, expected - len(buffer)))
            if not chunk:
                continue
            buffer.extend(chunk)
            # Some USB adapters echo the command before the servo replies.
            while buffer:
                start = buffer.find(b"\xff\xff")
                if start < 0:
                    buffer.clear()
                    break
                if start:
                    del buffer[:start]
                if buffer.startswith(request):
                    del buffer[:len(request)]
                    continue
                if len(buffer) < expected:
                    break
                candidate = bytes(buffer[:expected])
                try:
                    return self.parse_reply(candidate, servo_id, response_bytes)
                except ServoError:
                    del buffer[0]
        raise ServoError(f"ID {servo_id}: no valid reply before {self.timeout:.3f}s timeout")

    def ping(self, servo_id: int) -> None:
        self._exchange(servo_id, 0x01, b"", 0)

    def read(self, servo_id: int, address: int, count: int) -> bytes:
        return self._exchange(servo_id, 0x02, bytes((address, count)), count)

    def model(self, servo_id: int) -> int:
        return int.from_bytes(self.read(servo_id, 3, 2), "little")

    def sample(self, ids: tuple[int, ...]) -> ServoSample:
        values = {servo_id: int.from_bytes(self.read(servo_id, 56, 2), "little") for servo_id in ids}
        return ServoSample(values, time.monotonic())
