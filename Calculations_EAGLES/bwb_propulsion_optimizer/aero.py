"""
Aerodynamik-/Missionsmodell des Flugzeugs (Blended Wing Body).

Aufgabe: Aus den Eckdaten (Masse, Gleitzahl, Reisegeschwindigkeit) den
benoetigten *Schub* (= Widerstand im stationaeren Horizontalflug) bei einer
beliebigen Fluggeschwindigkeit berechnen.

Es gibt zwei Modi:

1) "glide"-Modus (Default, braucht nur die vorhandenen Eckdaten):
   - Im Reiseflug gilt T = D = W / (L/D).
   - Der Reisewiderstand wird in einen parasitaeren Anteil (~ v^2) und einen
     induzierten Anteil (~ 1/v^2) aufgeteilt. Standardannahme: Reiseflug liegt
     nahe dem besten Gleiten, dort ist parasitaerer ~ induzierter Widerstand
     (cd0_fraction = 0.5). Damit ergibt sich eine physikalisch sinnvolle
     Widerstand-ueber-Geschwindigkeit-Kurve ohne Kenntnis von Flaeche/Streckung.

2) "polar"-Modus (genauer, wenn Fluegelflaeche S, Streckung AR, Oswald e
   bekannt sind): klassische Drag-Polare.

Alle Einheiten SI (kg, m, s, N).
"""

from __future__ import annotations
from dataclasses import dataclass
import math

G = 9.80665  # m/s^2


@dataclass
class Airframe:
    mass_kg: float
    v_cruise: float
    glide_ratio: float = 10.0          # L/D im Reiseflug
    cd0_fraction: float = 0.5          # Anteil parasitaerer Widerstand im Reiseflug
    rho: float = 1.225                 # Luftdichte [kg/m^3]
    # Optional fuer den genaueren Polaren-Modus:
    wing_area: float | None = None     # S [m^2]
    aspect_ratio: float | None = None  # AR [-]
    oswald_e: float = 0.85

    @property
    def weight(self) -> float:
        return self.mass_kg * G

    def drag(self, v: float) -> float:
        """Widerstand [N] bei Geschwindigkeit v [m/s] im Horizontalflug (L = W)."""
        if self.wing_area and self.aspect_ratio:
            return self._drag_polar(v)
        return self._drag_glide(v)

    # gleicher Wert, aber sprechender Name fuer den Antrieb:
    def thrust_required(self, v: float) -> float:
        return self.drag(v)

    # ---- Modus 1: aus Gleitzahl -----------------------------------------
    def _drag_glide(self, v: float) -> float:
        d_cruise = self.weight / self.glide_ratio
        dp_cruise = self.cd0_fraction * d_cruise          # parasitaer @ v_cruise
        di_cruise = (1.0 - self.cd0_fraction) * d_cruise  # induziert  @ v_cruise
        dp = dp_cruise * (v / self.v_cruise) ** 2          # ~ v^2
        di = di_cruise * (self.v_cruise / v) ** 2          # ~ 1/v^2 (W = const)
        return dp + di

    # ---- Modus 2: echte Drag-Polare -------------------------------------
    def _drag_polar(self, v: float) -> float:
        q = 0.5 * self.rho * v * v
        S = self.wing_area
        AR = self.aspect_ratio
        # CD0 so kalibriert, dass im Reiseflug die geforderte Gleitzahl entsteht.
        cl_cr = self.weight / (0.5 * self.rho * self.v_cruise ** 2 * S)
        k = 1.0 / (math.pi * self.oswald_e * AR)
        # L/D = CL / (CD0 + k CL^2) -> CD0 aus geforderter Gleitzahl
        cd0 = cl_cr / self.glide_ratio - k * cl_cr ** 2
        cd0 = max(cd0, 1e-4)
        cl = self.weight / (q * S)
        cd = cd0 + k * cl ** 2
        return cd * q * S

    def best_glide_speed(self) -> float:
        """Geschwindigkeit minimalen Widerstands im glide-Modus."""
        # min(Dp*(v/vc)^2 + Di*(vc/v)^2) -> v = vc * (Di/Dp)^(1/4)
        ratio = (1 - self.cd0_fraction) / self.cd0_fraction
        return self.v_cruise * ratio ** 0.25
