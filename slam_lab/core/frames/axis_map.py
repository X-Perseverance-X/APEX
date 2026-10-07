"""Validated signed permutation from sensor raw coordinates to base_link."""

from __future__ import annotations


class AxisMap:
    def __init__(self, base_axes_from_raw: list[str]) -> None:
        if len(base_axes_from_raw) != 3:
            raise ValueError("expected three base axes")
        self.indices: list[int] = []
        self.signs: list[int] = []
        for token in base_axes_from_raw:
            if len(token) != 2 or token[0] not in "+-" or token[1] not in "xyz":
                raise ValueError(f"invalid axis token: {token}")
            self.indices.append("xyz".index(token[1]))
            self.signs.append(1 if token[0] == "+" else -1)
        if len(set(self.indices)) != 3:
            raise ValueError("each raw axis must be used exactly once")
        parity = 1 if self.indices in ([0, 1, 2], [1, 2, 0], [2, 0, 1]) else -1
        if parity * self.signs[0] * self.signs[1] * self.signs[2] != 1:
            raise ValueError("axis mapping must be right handed")

    def apply(self, raw: tuple[float, float, float]) -> tuple[float, float, float]:
        return tuple(self.signs[i] * raw[self.indices[i]] for i in range(3))  # type: ignore[return-value]

