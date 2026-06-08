"""
Minimale Fortschrittsanzeige ohne Zusatzabhaengigkeit.

Funktioniert in Terminals und in der PyCharm-Run-Konsole (Carriage-Return).
Updates sind gedrosselt (max. ~10/s), damit das Drucken die Rechnung nicht
ausbremst. `enabled=False` schaltet sie still.
"""

from __future__ import annotations
import sys
import time


class ProgressBar:
    def __init__(self, total: int, prefix: str = "Optimiere",
                 width: int = 32, enabled: bool = True, stream=None):
        self.total = max(int(total), 1)
        self.n = 0
        self.width = width
        self.prefix = prefix
        self.enabled = enabled
        self.stream = stream or sys.stdout
        self._t0 = time.time()
        self._last = 0.0

    def _render(self):
        frac = min(self.n / self.total, 1.0)
        filled = int(self.width * frac)
        bar = "#" * filled + "-" * (self.width - filled)
        el = time.time() - self._t0
        self.stream.write(
            f"\r{self.prefix} [{bar}] {frac*100:3.0f}% "
            f"({self.n}/{self.total}) {el:4.1f}s")
        self.stream.flush()

    def update(self, k: int = 1):
        self.n += k
        if not self.enabled:
            return
        now = time.time()
        if self.n < self.total and now - self._last < 0.1:
            return                      # drosseln
        self._last = now
        self._render()

    def close(self):
        if self.enabled:
            self.n = min(self.n, self.total)
            self._render()
            self.stream.write("\n")
            self.stream.flush()

    # Komfort: als Kontextmanager nutzbar
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
