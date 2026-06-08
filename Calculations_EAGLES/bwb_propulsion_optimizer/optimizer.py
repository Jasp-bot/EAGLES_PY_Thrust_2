"""
Optimizer: probiert alle (gefilterten) Motor x Propeller-Kombinationen durch,
rankt nach Cruise-Effizienz und liefert die Auslegungs-Entscheidungsgroessen
aus dem STEVE-Verlauf mit.

Laufzeit: ueber kv_min/kv_max (Motoren) und dmin_in/dmax_in (Propeller) laesst
sich der Suchraum eingrenzen. Eine Fortschrittsanzeige zeigt den Durchlauf.

Methodische Trennung (aus dem STEVE-Verlauf):
  - Cruise-Effizienz/Endurance gegen den ECHTEN aerodyn. Widerstand (L/D).
  - Reserve (Startschub, Vmax) gegen die Vollgas-Faehigkeit.
"""

from __future__ import annotations
from dataclasses import dataclass

from .aero import Airframe
from .battery import Battery
from .motor import Motor
from .propeller import BasePropeller, IN2M
from .filters import Filters
from .progress import ProgressBar
from .solver import (evaluate, OperatingPoint, vmax as solve_vmax,
                     static_max_thrust, full_throttle_point, endurance,
                     ideal_kv)


@dataclass
class Candidate:
    motor: Motor
    prop: BasePropeller
    cruise: OperatingPoint
    score: float                 # el. Cruise-Leistung [W] (kleiner = besser)
    static_thrust: float         # Vollgas-Standschub [N]
    load_point: float            # Auslegungsschub / Standschub  [0..1]
    vmax: float | None           # [m/s]
    bungee_thrust: float | None  # Vollgas-Schub am Bungee-Exit [N]
    kv_ideal: float              # ideales Kv aus Cruise-Drehzahl
    endurance: dict | None       # {time_h, range_km} falls Akku gegeben
    notes: tuple


def filter_components(motors, props, filters: Filters):
    """Wendet ein Filters-Objekt auf geladene Listen an. Gibt (motors, props)."""
    return ([m for m in motors if filters.motor_ok(m)],
            [p for p in props if filters.prop_ok(p)])


def optimize(motors: list[Motor], props: list[BasePropeller],
             airframe: Airframe, battery: Battery,
             v_max_target: float | None = 30.0,
             pusher_factor: float = 0.92,
             kv_min: float | None = None,
             kv_max: float | None = None,
             dmin_in: float | None = None,
             dmax_in: float | None = None,
             filters: Filters | None = None,
             design_thrust: float | None = None,
             bungee_speed: float = 14.0,
             battery_wh: float | None = None,
             reserve: float = 0.20,
             load_band=(0.40, 0.70),
             heavy_metrics_top: int | None = None,
             progress: bool = True,
             top_n: int = 10) -> list[Candidate]:

    if filters is None:
        filters = Filters(kv_min=kv_min, kv_max=kv_max,
                          dmin_in=dmin_in, dmax_in=dmax_in)
    motors, props = filter_components(motors, props, filters)
    total = len(motors) * len(props)
    bar = ProgressBar(total, prefix="Optimiere", enabled=progress)

    # --- Phase 1: billige Cruise-Bewertung fuer ALLE Kombinationen ---------
    feasible = []   # (p_elec, cruise, motor, prop, kv_ideal)
    for p in props:
        for m in motors:
            bar.update(1)
            cr = evaluate(p, m, airframe, battery, airframe.v_cruise, pusher_factor)
            if not cr.feasible:
                continue
            kvi = ideal_kv(cr.rpm, battery.v_design)
            feasible.append((cr.p_elec, cr, m, p, kvi))
    bar.close()

    feasible.sort(key=lambda t: t[0])      # nach el. Cruise-Leistung sortieren

    # --- Phase 2: teure Groessen nur fuer die besten K (oder alle) ---------
    rho = airframe.rho
    results: list[Candidate] = []
    for i, (pe, cr, m, p, kvi) in enumerate(feasible):
        heavy = (heavy_metrics_top is None) or (i < heavy_metrics_top)
        if not heavy:
            results.append(Candidate(
                motor=m, prop=p, cruise=cr, score=cr.p_elec,
                static_thrust=0.0, load_point=0.0, vmax=None,
                bungee_thrust=None, kv_ideal=kvi, endurance=None, notes=()))
            continue
        tstat = static_max_thrust(p, m, battery, rho)
        ref_thrust = design_thrust if design_thrust else cr.thrust_prop
        load = (ref_thrust / tstat) if tstat > 0 else 0.0
        vm = solve_vmax(p, m, airframe, battery, pusher_factor)
        bp = full_throttle_point(p, m, battery, bungee_speed, rho)
        end = endurance(cr.p_elec, battery_wh, reserve, airframe.v_cruise)
        notes = []
        if tstat and load < load_band[0]:
            notes.append(f"Teillast {load*100:.0f}% (<{load_band[0]*100:.0f}%)")
        elif load > load_band[1]:
            notes.append(f"hohe Last {load*100:.0f}%")
        if vm is None:
            notes.append("Vmax<cruise")
        elif v_max_target and vm < v_max_target - 0.5:
            notes.append(f"Vmax {vm:.0f}<{v_max_target:.0f}")
        elif v_max_target and vm > v_max_target + 6:
            notes.append(f"Vmax {vm:.0f} (Kv hoch)")
        if m.kv > 1.6 * kvi:
            notes.append("Kv>>ideal")
        results.append(Candidate(
            motor=m, prop=p, cruise=cr, score=cr.p_elec,
            static_thrust=tstat, load_point=load, vmax=vm,
            bungee_thrust=(bp["thrust"] if bp else None),
            kv_ideal=kvi, endurance=end, notes=tuple(notes)))

    return results if top_n is None else results[:top_n]


def format_table(cands: list[Candidate]) -> str:
    if not cands:
        return ("Keine machbare Kombination im Cruise gefunden. Moegliche "
                "Ursachen: Filter (Kv-/Durchmesserbereich) zu eng, "
                "Spannungsgrenze (12S), Propeller zu klein/gross fuer den "
                "Schubbedarf, oder Motor-Grenzen (i_max/p_max) zu streng.")
    hdr = (f"{'#':>2} {'Motor':<11} {'Prop':<11} {'ηges':>4} {'P_cr':>5} "
           f"{'I':>4} {'Vmax':>4} {'Last':>5} {'Kv':>4} {'Kvid':>4} "
           f"{'Tstat':>5}")
    lines = [hdr, "-" * len(hdr)]
    for i, c in enumerate(cands, 1):
        cr = c.cruise
        vm = f"{c.vmax:.0f}" if c.vmax else "--"
        lines.append(
            f"{i:>2} {c.motor.name:<11} {c.prop.name:<11} "
            f"{cr.eta_total:4.2f} {cr.p_elec:5.0f} {cr.current:4.0f} "
            f"{vm:>4} {c.load_point*100:4.0f}% {c.motor.kv:4.0f} "
            f"{c.kv_ideal:4.0f} {c.static_thrust:5.0f}")
        if c.notes:
            lines.append(f"   -> {' | '.join(c.notes)}")
    return "\n".join(lines)
