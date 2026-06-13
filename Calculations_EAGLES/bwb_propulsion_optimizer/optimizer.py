"""
Optimizer: probiert alle (gefilterten) Motor x Propeller-Kombinationen durch und
rankt sie. Ranking-Logik (Stand 0.6):

  - Basis ist die el. Cruise-Leistung (kleiner = effizienter).
  - WEICHE Lastpunkt-Strafe: faellt der Motor-Lastpunkt im Cruise unter
    load_min (Default 30 %, Uebergang +-load_tol), wird der Score multiplikativ
    bestraft -> verhindert chronischen Teillast-/10%-Throttle-Betrieb.
  - HARTES Vmax-Fenster: eine Kombination muss v_max bei Vollgas erreichen und
    darf hoechstens v_max + vmax_cap_margin (Default 10 m/s) schnell sein.
    Damit wird der Loesungsraum nach oben begrenzt (keine ueberdimensionierten,
    massiv ueberdrehenden Kombinationen).
  - Antriebsmasse (Motor + Rotor) ist Anzeige + Tiebreak: bei quasi gleichem
    Score gewinnt die leichtere Kombination.

Vmax und Lastpunkt sind damit Filter-/Ranking-Kriterien und werden fuer ALLE
machbaren Kombinationen berechnet (zweiter Fortschrittsbalken).
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
    score: float                 # Ranking-Score (penalisiert) - kleiner = besser
    static_thrust: float         # Vollgas-Standschub [N]
    load_point: float            # Auslegungsschub / Standschub  [0..1]
    vmax: float | None           # [m/s]
    bungee_thrust: float | None  # Vollgas-Schub am Bungee-Exit [N]
    kv_ideal: float              # ideales Kv aus Cruise-Drehzahl
    endurance: dict | None       # {time_h, range_km} falls Akku gegeben
    mass_total: float            # Motor + Rotor [kg]
    penalty: float               # Score-Faktor aus der Lastpunkt-Strafe (>=1)
    notes: tuple


def filter_components(motors, props, filters: Filters):
    """Wendet ein Filters-Objekt auf geladene Listen an. Gibt (motors, props)."""
    return ([m for m in motors if filters.motor_ok(m)],
            [p for p in props if filters.prop_ok(p)])


def _load_penalty(load, tstat, load_min, load_tol, weight):
    """Weiche Strafe: 0 ab (load_min+tol), voll ab (load_min-tol), linear dazw."""
    if not weight or load_min is None or tstat <= 0:
        return 1.0, 0.0
    hi, lo = load_min + load_tol, load_min - load_tol
    sf = (hi - load) / (hi - lo) if hi > lo else (1.0 if load < load_min else 0.0)
    sf = min(max(sf, 0.0), 1.0)
    return 1.0 + weight * sf, sf


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
             # --- Vmax-Fenster ---
             vmax_reach: bool = True,            # Vmax >= v_max erforderlich
             vmax_cap_margin: float | None = 10.0,   # Vmax <= v_max + margin
             # --- weicher Mindest-Lastpunkt ---
             load_min: float | None = 0.30,
             load_tol: float = 0.05,
             load_penalty_weight: float = 0.5,
             # --- Masse ---
             mass_tiebreak: bool = True,
             heavy_metrics_top: int | None = None,
             progress: bool = True,
             top_n: int = 10) -> list[Candidate]:

    if filters is None:
        filters = Filters(kv_min=kv_min, kv_max=kv_max,
                          dmin_in=dmin_in, dmax_in=dmax_in)
    motors, props = filter_components(motors, props, filters)
    total = len(motors) * len(props)

    # --- Phase 1: billige Cruise-Bewertung fuer ALLE Kombinationen ---------
    bar = ProgressBar(total, prefix="Cruise-Scan", enabled=progress)
    feasible = []   # (cruise, motor, prop, kv_ideal)
    for p in props:
        for m in motors:
            bar.update(1)
            cr = evaluate(p, m, airframe, battery, airframe.v_cruise, pusher_factor)
            if not cr.feasible:
                continue
            feasible.append((cr, m, p, ideal_kv(cr.rpm, battery.v_design)))
    bar.close()

    # --- Phase 2: Vmax + Lastpunkt fuer ALLE machbaren (Filter + Ranking) ---
    rho = airframe.rho
    cap_hi = (v_max_target + vmax_cap_margin) if (v_max_target is not None
              and vmax_cap_margin is not None) else None
    bar2 = ProgressBar(len(feasible), prefix="Vmax/Last ", enabled=progress)
    kept = []
    for cr, m, p, kvi in feasible:
        bar2.update(1)
        tstat = static_max_thrust(p, m, battery, rho)
        ref_thrust = design_thrust if design_thrust else cr.thrust_prop
        load = (ref_thrust / tstat) if tstat > 0 else 0.0
        vm = solve_vmax(p, m, airframe, battery, pusher_factor)
        # ---- hartes Vmax-Fenster ----
        if v_max_target is not None:
            if vmax_reach and (vm is None or vm < v_max_target - 1e-6):
                continue                         # erreicht v_max nicht
            if cap_hi is not None and vm is not None and vm > cap_hi + 1e-6:
                continue                         # zu schnell (Obergrenze)
        # ---- weiche Lastpunkt-Strafe ----
        penalty, sf = _load_penalty(load, tstat, load_min, load_tol,
                                    load_penalty_weight)
        mass_total = (m.mass_kg or 0.0) + (getattr(p, "mass_kg", 0.0) or 0.0)
        kept.append(dict(cr=cr, m=m, p=p, kvi=kvi, tstat=tstat, load=load,
                         vm=vm, score=cr.p_elec * penalty, penalty=penalty,
                         mass=mass_total))
    bar2.close()

    # --- Sortierung: Score, bei quasi-Gleichstand leichtere Kombi zuerst ----
    if kept:
        smin = min(k["score"] for k in kept)
        tol_abs = max(1.0, 0.01 * smin)          # ~1 %-Bucket fuer den Tiebreak
        if mass_tiebreak:
            kept.sort(key=lambda k: (round(k["score"] / tol_abs), k["mass"]))
        else:
            kept.sort(key=lambda k: k["score"])

    # --- Phase 3: billige Extras (Startschub, Endurance) fuer Top-K ---------
    results: list[Candidate] = []
    for i, k in enumerate(kept):
        heavy = (heavy_metrics_top is None) or (i < heavy_metrics_top)
        bp = (full_throttle_point(k["p"], k["m"], battery, bungee_speed, rho)
              if heavy else None)
        end = (endurance(k["cr"].p_elec, battery_wh, reserve, airframe.v_cruise)
               if heavy else None)

        load, tstat, vm = k["load"], k["tstat"], k["vm"]
        notes = []
        if tstat and load_min is not None and load < load_min:
            notes.append(f"Teillast {load*100:.0f}% (<{load_min*100:.0f}%, "
                         f"Score x{k['penalty']:.2f})")
        elif load > load_band[1]:
            notes.append(f"hohe Last {load*100:.0f}%")
        if vm is not None and v_max_target:
            if vm > v_max_target + (vmax_cap_margin or 99) - 1.0:
                notes.append(f"Vmax {vm:.0f} (nahe Cap)")
        if k["m"].kv > 1.6 * k["kvi"]:
            notes.append("Kv>>ideal")

        results.append(Candidate(
            motor=k["m"], prop=k["p"], cruise=k["cr"], score=k["score"],
            static_thrust=tstat, load_point=load, vmax=vm,
            bungee_thrust=(bp["thrust"] if bp else None),
            kv_ideal=k["kvi"], endurance=end, mass_total=k["mass"],
            penalty=k["penalty"], notes=tuple(notes)))

    return results if top_n is None else results[:top_n]


def format_table(cands: list[Candidate]) -> str:
    if not cands:
        return ("Keine Kombination erfuellt alle Bedingungen. Moegliche Ursachen: "
                "Vmax-Fenster (v_max..v_max+Cap) zu eng oder keine Kombi erreicht "
                "v_max; Filter (Kv/Durchmesser/Hersteller) zu streng; "
                "Spannung/12S; Motor-Grenzen (i_max/p_max). Tipp: Zellzahl, "
                "vmax_cap_margin oder die Filter anpassen.")
    hdr = (f"{'#':>2} {'Motor':<11} {'Prop':<11} {'ηges':>4} {'P_cr':>5} "
           f"{'I':>4} {'Vmax':>4} {'Last':>5} {'Masse':>6} {'Kv':>4} "
           f"{'Kvid':>4} {'Tstat':>5}")
    lines = [hdr, "-" * len(hdr)]
    for i, c in enumerate(cands, 1):
        cr = c.cruise
        vm = f"{c.vmax:.0f}" if c.vmax else "--"
        lines.append(
            f"{i:>2} {c.motor.name:<11} {c.prop.name:<11} "
            f"{cr.eta_total:4.2f} {cr.p_elec:5.0f} {cr.current:4.0f} "
            f"{vm:>4} {c.load_point*100:4.0f}% {c.mass_total*1000:5.0f}g "
            f"{c.motor.kv:4.0f} {c.kv_ideal:4.0f} {c.static_thrust:5.0f}")
        if c.notes:
            lines.append(f"   -> {' | '.join(c.notes)}")
    return "\n".join(lines)
