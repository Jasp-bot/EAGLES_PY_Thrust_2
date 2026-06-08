"""
Propeller-Modell.

Schub und Leistung ueber die dimensionslosen Beiwerte (Standard-APC-Definition):
    J  = V / (n * D)            (n in Umdrehungen/s, D Durchmesser in m)
    T  = CT(J) * rho * n^2 * D^4
    P  = CP(J) * rho * n^3 * D^5
    eta_prop = J * CT / CP = T*V / P

Drei Klassen:
  BasePropeller       gemeinsamer Drehzahl-Loeser (brentq), nutzt self.CT/self.CP
  MeasuredPropeller   CT/CP aus echten Messdaten (PyThrust/APC-Performance-CSV)
                      -> der belastbare Weg, RPM-abhaengig (Reynolds) interpoliert
  EstimatedPropeller  grobe Naeherung aus Durchmesser/Steigung/Blattzahl,
                      nur als Rueckfall, wenn keine Messdaten vorliegen
"""

from __future__ import annotations
from dataclasses import dataclass, field
import math
import numpy as np
from scipy.optimize import brentq

IN2M = 0.0254


class BasePropeller:
    name: str
    diameter_m: float
    pitch_m: float
    blades: int
    mass_kg: float
    manufacturer: str = ""

    # --- von Subklassen bereitzustellen ---
    def CT(self, J: float, rpm: float | None = None) -> float:
        raise NotImplementedError

    def CP(self, J: float, rpm: float | None = None) -> float:
        raise NotImplementedError

    @property
    def j_t0(self) -> float:
        raise NotImplementedError

    def eta(self, J: float, rpm: float | None = None) -> float:
        cp = self.CP(J, rpm)
        return J * self.CT(J, rpm) / cp if cp > 1e-9 else 0.0

    # --- gemeinsame Physik ---
    @property
    def disk_area(self) -> float:
        return math.pi * (self.diameter_m / 2.0) ** 2

    @property
    def pitch_ratio(self) -> float:
        return self.pitch_m / self.diameter_m

    def thrust(self, n_rps: float, v: float, rho: float) -> float:
        if n_rps <= 0:
            return 0.0
        J = v / (n_rps * self.diameter_m)
        return self.CT(J, n_rps * 60.0) * rho * n_rps ** 2 * self.diameter_m ** 4

    def solve_rps_for_thrust(self, T: float, v: float, rho: float):
        """T = CT(J) rho n^2 D^4 nach n (rev/s) loesen. dict oder None."""
        D = self.diameter_m
        n_lo = (v / (D * self.j_t0)) * 1.0001 if v > 1e-6 else 1e-3
        n_hi = max(n_lo * 60.0, 800.0)          # bis ~48000 rpm
        f = lambda n: self.thrust(n, v, rho) - T
        if f(n_lo) > 0:
            n_lo = 1e-3
            if f(n_lo) > 0:
                return None
        if f(n_hi) < 0:
            return None
        n = brentq(f, n_lo, n_hi, xtol=1e-4, rtol=1e-6, maxiter=200)
        J = v / (n * D)
        rpm = n * 60.0
        P = self.CP(J, rpm) * rho * n ** 3 * D ** 5
        Q = P / (2 * math.pi * n)
        return dict(n_rps=n, rpm=rpm, J=J, P_shaft=P, torque=Q,
                    eta_prop=self.eta(J, rpm))


# ============================================================ Messdaten
class MeasuredPropeller(BasePropeller):
    """CT/CP aus einer Performance-Tabelle (rpm, J, Ct, Cp, eta)."""

    def __init__(self, name, diameter_m, pitch_m, blades=2, mass_kg=0.0,
                 rpm=None, J=None, ct=None, cp=None, eff=None, manufacturer=""):
        self.name = name
        self.diameter_m = diameter_m
        self.pitch_m = pitch_m
        self.blades = blades
        self.mass_kg = mass_kg
        self.manufacturer = manufacturer
        self.estimated = False
        rpm = np.asarray(rpm, float)
        J = np.asarray(J, float)
        ct = np.asarray(ct, float)
        cp = np.asarray(cp, float)
        eff = np.asarray(eff, float) if eff is not None else None
        # pro RPM-Block sortierte (J, Ct, Cp, eta)-Kurven ablegen
        self._blocks = {}
        for rp in np.unique(rpm):
            m = rpm == rp
            order = np.argsort(J[m])
            self._blocks[float(rp)] = (
                J[m][order], ct[m][order], cp[m][order],
                (eff[m][order] if eff is not None else None))
        self._rpm_levels = np.array(sorted(self._blocks))
        # globaler Schub-Nulldurchgang (max J mit Ct>0) fuer das Bracketing
        pos = J[ct > 0]
        self._jt0 = float(pos.max()) if pos.size else float(J.max())

    @property
    def j_t0(self) -> float:
        return self._jt0

    def _interp_block(self, rp, J, which):
        Jb, ctb, cpb, effb = self._blocks[rp]
        if which == "ct":
            return float(np.interp(J, Jb, ctb))
        if which == "cp":
            return float(np.interp(J, Jb, cpb))
        if which == "eff" and effb is not None:
            return float(np.interp(J, Jb, effb))
        return None

    def _eval(self, J, rpm, which):
        levels = self._rpm_levels
        if rpm is None:
            rpm = levels[len(levels) // 2]           # repraesentativer Block
        rpm = min(max(rpm, levels[0]), levels[-1])    # auf Datenbereich klemmen
        i = np.searchsorted(levels, rpm)
        if i == 0:
            return self._interp_block(levels[0], J, which)
        if i >= len(levels):
            return self._interp_block(levels[-1], J, which)
        lo, hi = levels[i - 1], levels[i]
        w = (rpm - lo) / (hi - lo)
        a = self._interp_block(lo, J, which)
        b = self._interp_block(hi, J, which)
        if a is None or b is None:
            return None
        return a + w * (b - a)

    def CT(self, J, rpm=None):
        return self._eval(J, rpm, "ct")

    def CP(self, J, rpm=None):
        return max(self._eval(J, rpm, "cp"), 1e-6)

    def eta(self, J, rpm=None):
        e = self._eval(J, rpm, "eff")
        if e is not None:
            return e
        return super().eta(J, rpm)

    @classmethod
    def from_apc_csv(cls, path, name, diameter_m, pitch_m, blades=2, mass_kg=0.0,
                     manufacturer=""):
        import csv
        rpm, J, ct, cp, eff = [], [], [], [], []
        with open(path, newline="", encoding="utf-8-sig") as fh:
            for r in csv.DictReader(fh):
                rpm.append(float(r["rpm"]))
                J.append(float(r["advance_ratio"]))
                ct.append(float(r["thrust_coeff"]))
                cp.append(float(r["power_coeff"]))
                eff.append(float(r["efficiency"]))
        return cls(name, diameter_m, pitch_m, blades, mass_kg,
                   rpm=rpm, J=J, ct=ct, cp=cp, eff=eff, manufacturer=manufacturer)


# ============================================================ Schaetzer
@dataclass
class EstimatedPropeller(BasePropeller):
    name: str
    diameter_m: float
    pitch_m: float
    blades: int = 2
    mass_kg: float = 0.0
    eta_max: float = 0.78
    manufacturer: str = ""
    estimated: bool = field(default=True, repr=False)

    def __post_init__(self):
        p = self.pitch_ratio
        bf = (self.blades / 2.0) ** 0.72
        self._jt0 = max(0.30, 1.0 * p)
        self._ct0 = (0.085 + 0.045 * p) * bf
        self._xm = 0.60
        self._A = 1.0
        self._B = (1.0 - self._xm) / self._xm
        self._shape_peak = (self._xm ** self._A) * ((1 - self._xm) ** self._B)

    @property
    def j_t0(self):
        return self._jt0

    def CT(self, J, rpm=None):
        x = J / self._jt0
        return self._ct0 * (1.0 - x) if x < 1.0 else 0.0

    def eta(self, J, rpm=None):
        x = J / self._jt0
        if x <= 0.0 or x >= 1.0:
            return 0.0
        shape = (x ** self._A) * ((1 - x) ** self._B)
        return self.eta_max * shape / self._shape_peak

    def CP(self, J, rpm=None):
        e = self.eta(J, rpm)
        if e <= 1e-4 or J <= 1e-6:
            jj = 1e-3 * self._jt0
            e0 = self.eta(jj)
            return max(jj * self.CT(jj) / e0, 1e-5) if e0 > 0 else 1e-3
        return max(J * self.CT(J) / e, 1e-6)


# Komfort-Konstruktor mit Zoll-Angaben (Schaetzer)
def estimated_from_inches(name, diameter_in, pitch_in, blades=2, mass_kg=0.0,
                          eta_max=0.78):
    return EstimatedPropeller(name, diameter_in * IN2M, pitch_in * IN2M,
                              blades, mass_kg, eta_max)
