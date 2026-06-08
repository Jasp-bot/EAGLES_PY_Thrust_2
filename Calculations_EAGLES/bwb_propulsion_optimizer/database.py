"""
Datenbank-Anbindung.

Unterstuetzt jetzt das ECHTE PyThrust-Format:

  Motor  = JSON-Datei, z. B.
    {"id","name","manufacturer","kv","resistance","io","max_current",
     "weight_g","max_power","io_voltage"}

  Propeller = JSON-Datei mit Metadaten + Verweis auf eine Performance-CSV:
    {"id","manufacturer","model","diameter_in","pitch_in","blade_count",
     "data_csv":"<datei>.csv"}
  Die CSV ist eine APC-Performance-Tabelle mit Spalten
    rpm, advance_ratio, thrust_coeff, power_coeff, efficiency, ...
  -> daraus wird ein MeasuredPropeller mit gemessenen Ct(J)/Cp(J) gebaut.

Zusaetzlich: tolerante CSV-Loader (Alias-Mapping) und synthetische
Beispieldaten zum Ausprobieren.
"""

from __future__ import annotations
import csv
import json
import os
import glob
from .motor import Motor
from .propeller import (MeasuredPropeller, EstimatedPropeller,
                        estimated_from_inches, IN2M)


# ============================================================ PyThrust JSON
def load_motor_json(path: str) -> Motor:
    with open(path, encoding="utf-8") as fh:
        d = json.load(fh)
    mass_g = d.get("weight_g")
    return Motor(
        name=str(d.get("name") or d.get("id")),
        kv=float(d["kv"]),
        Rm=float(d.get("resistance", d.get("Rm"))),
        I0=float(d.get("io", d.get("I0", 0.0))),
        io_voltage=(float(d["io_voltage"]) if d.get("io_voltage") else None),
        i_max=(float(d["max_current"]) if d.get("max_current") else None),
        p_max=(float(d["max_power"]) if d.get("max_power") else None),
        mass_kg=(mass_g / 1000.0 if mass_g else 0.0),
        manufacturer=str(d.get("manufacturer", "")))


def _resolve_csv(csv_name: str, json_path: str) -> str | None:
    base = os.path.dirname(json_path)
    # Kandidaten: Originalname, sowie Punkt->Unterstrich (Upload-Umbenennung)
    names = {csv_name, csv_name.replace(".", "_", csv_name.count(".") - 1)
             if csv_name.count(".") > 1 else csv_name}
    # generischer Ersatz aller Punkte ausser der Endung
    stem, ext = os.path.splitext(csv_name)
    names.add(stem.replace(".", "_") + ext)
    for nm in names:
        cand = os.path.join(base, nm)
        if os.path.exists(cand):
            return cand
    # letzter Versuch: per glob
    hits = glob.glob(os.path.join(base, stem.replace(".", "*") + ext))
    return hits[0] if hits else None


def load_propeller_json(path: str):
    with open(path, encoding="utf-8") as fh:
        d = json.load(fh)
    name = str(d.get("model") or d.get("id"))
    mfr = str(d.get("manufacturer", ""))
    d_in = d.get("diameter_in")
    p_in = d.get("pitch_in")
    diameter_m = float(d_in) * IN2M if d_in else float(d["diameter_m"])
    pitch_m = float(p_in) * IN2M if p_in else float(d["pitch_m"])
    blades = int(d.get("blade_count", d.get("blades", 2)))
    mass_kg = (float(d["weight_g"]) / 1000.0) if d.get("weight_g") else 0.0
    csv_name = d.get("data_csv")
    if csv_name:
        csv_path = _resolve_csv(csv_name, path)
        if csv_path:
            return MeasuredPropeller.from_apc_csv(
                csv_path, name, diameter_m, pitch_m, blades, mass_kg,
                manufacturer=mfr)
    # ohne Performance-CSV: Schaetzer
    return EstimatedPropeller(name, diameter_m, pitch_m, blades, mass_kg,
                              manufacturer=mfr)


def load_motors_dir(directory: str, kv_min: float | None = None,
                    kv_max: float | None = None, filters=None) -> list[Motor]:
    from .filters import Filters
    if filters is None:
        filters = Filters(kv_min=kv_min, kv_max=kv_max)
    out = []
    for p in sorted(glob.glob(os.path.join(directory, "**", "*.json"),
                              recursive=True)):
        try:
            with open(p, encoding="utf-8") as fh:
                d = json.load(fh)
            if "kv" not in d:                      # kein Motor-JSON
                continue
            if not filters.motor_json_ok(d):
                continue
        except (ValueError, TypeError, OSError):
            continue
        try:
            m = load_motor_json(p)
        except (KeyError, ValueError, TypeError):
            continue
        if filters.motor_ok(m):
            out.append(m)
    return out


def load_props_dir(directory: str, dmin_in: float | None = None,
                   dmax_in: float | None = None, filters=None) -> list:
    """Laedt Propeller-JSONs (rekursiv). Filter werden VOR dem teuren
    CSV-Parsen geprueft (Durchmesser/Steigung/Hersteller/Auswahl) -> spart
    viel Ladezeit bei grossen Katalogen."""
    from .filters import Filters
    if filters is None:
        filters = Filters(dmin_in=dmin_in, dmax_in=dmax_in)
    out = []
    for p in sorted(glob.glob(os.path.join(directory, "**", "*.json"),
                              recursive=True)):
        try:
            with open(p, encoding="utf-8") as fh:
                d = json.load(fh)
            if not ("diameter_in" in d or "diameter_m" in d):   # kein Prop-JSON
                continue
            if not filters.prop_json_ok(d):
                continue
        except (ValueError, TypeError, OSError):
            continue
        try:
            prop = load_propeller_json(p)          # erst hier wird die CSV gelesen
        except (KeyError, ValueError, TypeError):
            continue
        if filters.prop_ok(prop):
            out.append(prop)
    return out


# ============================================================ CSV (tolerant)
MOTOR_ALIASES = {
    "name":  ["name", "motor", "model", "bezeichnung", "id"],
    "kv":    ["kv", "kv_rpm_v", "kv_value"],
    "Rm":    ["resistance", "rm", "r_phase", "phase_resistance", "rm_ohm"],
    "I0":    ["io", "i0", "no_load_current", "idle_current"],
    "io_v":  ["io_voltage", "i0_voltage", "no_load_voltage"],
    "i_max": ["max_current", "i_max", "imax", "continuous_current"],
    "p_max": ["max_power", "p_max", "pmax", "power_max"],
    "mass":  ["weight_g", "mass_g", "mass", "weight", "mass_kg", "weight_kg"],
    "mfr":   ["manufacturer", "hersteller", "brand"],
}


def _pick(row_lc, names):
    for n in names:
        if n in row_lc and row_lc[n] not in ("", None):
            return row_lc[n]
    return None


def _f(x):
    return None if x in (None, "") else float(x)


def load_motors_csv(path: str) -> list[Motor]:
    motors = []
    with open(path, newline="", encoding="utf-8-sig") as fh:
        for row in csv.DictReader(fh):
            r = {k.strip().lower(): v for k, v in row.items() if k}
            kv = _f(_pick(r, MOTOR_ALIASES["kv"]))
            Rm = _f(_pick(r, MOTOR_ALIASES["Rm"]))
            I0 = _f(_pick(r, MOTOR_ALIASES["I0"]))
            name = _pick(r, MOTOR_ALIASES["name"])
            if not (name and kv and Rm is not None and I0 is not None):
                continue
            mass = _f(_pick(r, MOTOR_ALIASES["mass"])) or 0.0
            if mass > 5:                 # > 5 -> Gramm
                mass /= 1000.0
            motors.append(Motor(
                name=str(name), kv=kv, Rm=Rm, I0=I0,
                io_voltage=_f(_pick(r, MOTOR_ALIASES["io_v"])),
                i_max=_f(_pick(r, MOTOR_ALIASES["i_max"])),
                p_max=_f(_pick(r, MOTOR_ALIASES["p_max"])),
                mass_kg=mass,
                manufacturer=str(_pick(r, MOTOR_ALIASES["mfr"]) or "")))
    return motors


# ============================================================ Beispiel (synthetisch)
def example_motors() -> list[Motor]:
    """Synthetische 12S-Kandidaten (~15-17 kg Klasse). KEINE Herstellerdaten."""
    return [
        Motor("EX-130kv", 130, 0.090, 0.8, 10, 45, 2200, 0.430),
        Motor("EX-150kv", 150, 0.075, 0.9, 10, 50, 2400, 0.400),
        Motor("EX-170kv", 170, 0.060, 1.0, 10, 55, 2600, 0.380),
        Motor("EX-190kv", 190, 0.048, 1.2, 10, 60, 2800, 0.360),
        Motor("EX-220kv", 220, 0.038, 1.4, 10, 65, 3000, 0.340),
        Motor("EX-260kv", 260, 0.028, 1.8, 10, 80, 3400, 0.320),
    ]


def example_props() -> list:
    P = estimated_from_inches
    return [
        P("EX 16x10", 16, 10, 2, 0.060),
        P("EX 17x10", 17, 10, 2, 0.070),
        P("EX 18x12", 18, 12, 2, 0.085),
        P("EX 19x12", 19, 12, 2, 0.095),
        P("EX 19x13", 19, 13, 2, 0.100),
        P("EX 20x13", 20, 13, 2, 0.110),
    ]
