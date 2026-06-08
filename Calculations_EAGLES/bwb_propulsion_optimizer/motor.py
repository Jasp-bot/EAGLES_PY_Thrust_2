"""
BLDC-Motor-Modell (DC-Ersatzschaltbild).

Felder passend zum PyThrust-Schema (motor-JSON):
    kv          [rpm/V]   Drehzahlkonstante
    Rm          [Ohm]     Phasenwiderstand        (JSON: "resistance")
    I0          [A]       Leerlaufstrom           (JSON: "io")
    io_voltage  [V]       Spannung, bei der I0 gemessen wurde (JSON: "io_voltage")
    i_max       [A]       Dauerstrom-Grenze       (JSON: "max_current")
    p_max       [W]       Leistungsgrenze         (JSON: "max_power")
    mass_kg     [kg]      Masse                   (JSON: "weight_g" / 1000)

Beziehungen:
    Kt   = 60 / (2*pi*kv)          Drehmomentkonstante [Nm/A]
    I    = Q / Kt + I0(rpm)
    Uemf = rpm / kv
    Uklemme = Uemf + I*Rm
    Pelek   = Uklemme * I
    eta_motor = Pwelle / Pelek

Optional: I0 mit der Drehzahl skalieren (Eisenverluste ~ Drehzahl). Da I0 bei
io_voltage gemessen wurde, entspricht das der Leerlaufdrehzahl kv*io_voltage.
Standard ist konstantes I0 (etabliertes, einfaches Modell).
"""

from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass
class Motor:
    name: str
    kv: float                      # rpm/V
    Rm: float                      # Ohm
    I0: float                      # A
    io_voltage: float | None = None
    i_max: float | None = None     # A
    p_max: float | None = None     # W
    mass_kg: float = 0.0
    manufacturer: str = ""
    i0_speed_scaling: bool = False

    @property
    def Kt(self) -> float:
        return 60.0 / (2.0 * math.pi * self.kv)   # Nm/A

    def _i0_at(self, rpm: float) -> float:
        if self.i0_speed_scaling and self.io_voltage:
            rpm_ref = self.kv * self.io_voltage
            if rpm_ref > 0:
                return self.I0 * rpm / rpm_ref
        return self.I0

    def max_rpm(self, voltage: float) -> float:
        """Naeherung der Vollgas-Leerlaufdrehzahl (ohne Last)."""
        return self.kv * voltage

    def operating_point(self, torque: float, rpm: float) -> dict:
        I = torque / self.Kt + self._i0_at(rpm)
        u_emf = rpm / self.kv
        u_term = u_emf + I * self.Rm
        p_elec = u_term * I
        p_shaft = torque * 2.0 * math.pi * rpm / 60.0
        eta = p_shaft / p_elec if p_elec > 0 else 0.0
        return dict(current=I, voltage=u_term, u_emf=u_emf,
                    p_elec=p_elec, p_shaft=p_shaft, eta_motor=eta)
