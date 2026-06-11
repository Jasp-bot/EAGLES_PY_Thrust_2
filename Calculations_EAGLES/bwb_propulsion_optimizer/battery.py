"""
Einfaches Akku-Modell (Bordspannung).

Standard: 12S. Spannung je nach Ladezustand. Fuer die Auslegung wird
typischerweise gegen die *nominale* Spannung gerechnet (Mittel ueber Entladung)
und gegen die *leere* Spannung als konservative Grenze geprueft.
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class Battery:
    cells: int = 12 # Cell voltage in c
    v_cell_full: float = 4.20
    v_cell_nominal: float = 3.70
    v_cell_empty: float = 3.50      # konservatives Spannungsende (Li-Ion)
    # Spannung, gegen die die Machbarkeit geprueft wird (Default: leer = worst case)
    design_state: str = "empty"     # "full" | "nominal" | "empty"

    @property
    def v_full(self):    return self.cells * self.v_cell_full
    @property
    def v_nominal(self): return self.cells * self.v_cell_nominal
    @property
    def v_empty(self):   return self.cells * self.v_cell_empty

    @property
    def v_design(self) -> float:
        return {"full": self.v_full,
                "nominal": self.v_nominal,
                "empty": self.v_empty}[self.design_state]
