"""
Filterkriterien fuer die Komponentenauswahl.

Ein `Filters`-Objekt buendelt alle Kriterien und kann
  - schon beim Laden greifen (JSON-Vorpruefung -> spart das CSV-Parsen), und
  - auf bereits geladene Listen angewandt werden (motor_ok / prop_ok).

Hersteller- und Auswahl-Listen erlauben MEHRERE Eintraege gleichzeitig
(z. B. zwei Hersteller vergleichen). Vergleich ist case-insensitive per
Teilstring ("t-motor" trifft "T-Motor"; "U8" trifft "U8 II KV150").
"""

from __future__ import annotations
from dataclasses import dataclass

IN2M = 0.0254


def _norm(s) -> str:
    return str(s if s is not None else "").strip().lower()


def _as_list(x):
    if x is None:
        return None
    if isinstance(x, (list, tuple, set)):
        return [str(v) for v in x if str(v).strip()]
    return [str(x)] if str(x).strip() else None


def _match_any(value, patterns) -> bool:
    """True, wenn `value` zu IRGENDEINEM Muster passt (Teilstring, case-insens.)."""
    if not patterns:
        return True
    v = _norm(value)
    return any(_norm(p) in v for p in patterns)


@dataclass
class Filters:
    # --- Motoren ---
    kv_min: float | None = None
    kv_max: float | None = None
    motor_mfr: list | None = None        # Hersteller (mehrere -> Vergleich)
    motor_select: list | None = None     # einzelne Motoren (Name/ID-Teilstring)
    motor_mass_max_g: float | None = None
    # --- Propeller ---
    dmin_in: float | None = None
    dmax_in: float | None = None
    pmin_in: float | None = None         # Steigung min [Zoll]
    pmax_in: float | None = None         # Steigung max [Zoll]
    pd_min: float | None = None          # P/D-Verhaeltnis min
    pd_max: float | None = None          # P/D-Verhaeltnis max
    blades: list | None = None           # erlaubte Blattzahlen, z. B. [2] oder [2,3]
    prop_mfr: list | None = None
    prop_select: list | None = None

    def __post_init__(self):
        for a in ("motor_mfr", "motor_select", "prop_mfr", "prop_select", "blades"):
            setattr(self, a, _as_list(getattr(self, a)))

    # ---------- auf geladene Objekte ----------
    def motor_ok(self, m) -> bool:
        if self.kv_min is not None and m.kv < self.kv_min:
            return False
        if self.kv_max is not None and m.kv > self.kv_max:
            return False
        if (self.motor_mass_max_g is not None and m.mass_kg
                and m.mass_kg * 1000.0 > self.motor_mass_max_g + 1e-6):
            return False
        if self.motor_mfr and not _match_any(getattr(m, "manufacturer", ""),
                                             self.motor_mfr):
            return False
        if self.motor_select and not _match_any(m.name, self.motor_select):
            return False
        return True

    def prop_ok(self, p) -> bool:
        d = p.diameter_m / IN2M
        pit = p.pitch_m / IN2M
        pd = (p.pitch_m / p.diameter_m) if p.diameter_m else 0.0
        if self.dmin_in is not None and d < self.dmin_in - 1e-6:
            return False
        if self.dmax_in is not None and d > self.dmax_in + 1e-6:
            return False
        if self.pmin_in is not None and pit < self.pmin_in - 1e-6:
            return False
        if self.pmax_in is not None and pit > self.pmax_in + 1e-6:
            return False
        if self.pd_min is not None and pd < self.pd_min - 1e-9:
            return False
        if self.pd_max is not None and pd > self.pd_max + 1e-9:
            return False
        if self.blades and int(p.blades) not in [int(b) for b in self.blades]:
            return False
        if self.prop_mfr and not _match_any(getattr(p, "manufacturer", ""),
                                            self.prop_mfr):
            return False
        if self.prop_select and not _match_any(p.name, self.prop_select):
            return False
        return True

    # ---------- JSON-Vorpruefung (vor dem CSV-Parsen) ----------
    def motor_json_ok(self, d: dict) -> bool:
        kv = d.get("kv")
        if kv is not None:
            kv = float(kv)
            if self.kv_min is not None and kv < self.kv_min:
                return False
            if self.kv_max is not None and kv > self.kv_max:
                return False
        if (self.motor_mass_max_g is not None and d.get("weight_g") is not None
                and float(d["weight_g"]) > self.motor_mass_max_g + 1e-6):
            return False
        if self.motor_mfr and not _match_any(d.get("manufacturer"), self.motor_mfr):
            return False
        if self.motor_select and not (_match_any(d.get("name"), self.motor_select)
                                      or _match_any(d.get("id"), self.motor_select)):
            return False
        return True

    def prop_json_ok(self, d: dict) -> bool:
        din = d.get("diameter_in")
        pin = d.get("pitch_in")
        if din is not None:
            din = float(din)
            if self.dmin_in is not None and din < self.dmin_in - 1e-6:
                return False
            if self.dmax_in is not None and din > self.dmax_in + 1e-6:
                return False
            if pin is not None:
                pin = float(pin)
                pd = pin / din if din else 0.0
                if self.pmin_in is not None and pin < self.pmin_in - 1e-6:
                    return False
                if self.pmax_in is not None and pin > self.pmax_in + 1e-6:
                    return False
                if self.pd_min is not None and pd < self.pd_min - 1e-9:
                    return False
                if self.pd_max is not None and pd > self.pd_max + 1e-9:
                    return False
        if (self.blades and d.get("blade_count") is not None
                and int(d["blade_count"]) not in [int(b) for b in self.blades]):
            return False
        if self.prop_mfr and not _match_any(d.get("manufacturer"), self.prop_mfr):
            return False
        if self.prop_select and not (_match_any(d.get("model"), self.prop_select)
                                     or _match_any(d.get("id"), self.prop_select)):
            return False
        return True

    # ---------- huebsche Zusammenfassung fuers Log ----------
    def summary(self) -> str:
        parts = []
        if self.kv_min is not None or self.kv_max is not None:
            parts.append(f"Kv {self.kv_min or 0:.0f}-{self.kv_max or 9999:.0f}")
        if self.dmin_in is not None or self.dmax_in is not None:
            parts.append(f'D {self.dmin_in or 0:.0f}-{self.dmax_in or 99:.0f}"')
        if self.pmin_in is not None or self.pmax_in is not None:
            parts.append(f'P {self.pmin_in or 0:.0f}-{self.pmax_in or 99:.0f}"')
        if self.pd_min is not None or self.pd_max is not None:
            parts.append(f"P/D {self.pd_min or 0:.2f}-{self.pd_max or 9:.2f}")
        if self.blades:
            parts.append(f"Blatt {self.blades}")
        if self.motor_mass_max_g:
            parts.append(f"Motor<= {self.motor_mass_max_g:.0f}g")
        if self.motor_mfr:
            parts.append(f"MotorHersteller {self.motor_mfr}")
        if self.prop_mfr:
            parts.append(f"PropHersteller {self.prop_mfr}")
        if self.motor_select:
            parts.append(f"nur Motoren {self.motor_select}")
        if self.prop_select:
            parts.append(f"nur Props {self.prop_select}")
        return " | ".join(parts) if parts else "keine"
