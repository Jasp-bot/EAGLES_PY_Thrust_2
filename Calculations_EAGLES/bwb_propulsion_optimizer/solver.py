"""
Betriebspunkt-Solver: Zelle -> Propeller -> Motor -> Akku.

Neben dem stationaeren Cruise-Punkt liefert dieses Modul die Groessen, die der
STEVE-Auslegungsverlauf als entscheidend identifiziert hat:
  - Motor-Lastpunkt (Cruise-Schub / Vollgas-Standschub)  -> Ideal 40-70 %
  - Vmax (drehzahl-/spannungslimitiert ueber Kv)          -> n_max ~ Kv*U
  - Startschub statisch und am Bungee-Exit
  - ideales Kv aus Cruise-Drehzahl und Bordspannung
"""

from __future__ import annotations
from dataclasses import dataclass
import math
from scipy.optimize import brentq

from .aero import Airframe
from .propeller import BasePropeller
from .motor import Motor
from .battery import Battery


@dataclass
class OperatingPoint:
    v: float
    thrust_aero: float
    thrust_prop: float
    rpm: float
    J: float
    eta_prop: float
    eta_motor: float
    eta_total: float
    current: float
    voltage: float
    p_shaft: float
    p_elec: float
    feasible: bool
    reasons: tuple


def evaluate(prop: BasePropeller, motor: Motor, airframe: Airframe,
             battery: Battery, v: float, pusher_factor: float = 0.92
             ) -> OperatingPoint:
    rho = airframe.rho
    t_aero = airframe.thrust_required(v)
    t_prop = t_aero / pusher_factor

    sol = prop.solve_rps_for_thrust(t_prop, v, rho)
    if sol is None:
        return OperatingPoint(v, t_aero, t_prop, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              False, ("Propeller kann Schub nicht liefern",))

    mp = motor.operating_point(sol["torque"], sol["rpm"])
    eta_total = (t_aero * v) / mp["p_elec"] if mp["p_elec"] > 0 else 0.0

    reasons = []
    if mp["voltage"] > battery.v_design:
        reasons.append(f"U={mp['voltage']:.1f}>U_bus={battery.v_design:.0f}V")
    if motor.i_max and mp["current"] > motor.i_max:
        reasons.append(f"I={mp['current']:.0f}>I_max={motor.i_max:.0f}A")
    if motor.p_max and mp["p_elec"] > motor.p_max:
        reasons.append(f"P={mp['p_elec']:.0f}>P_max={motor.p_max:.0f}W")
    if sol["J"] >= prop.j_t0:
        reasons.append("J>=J_T0")

    return OperatingPoint(
        v=v, thrust_aero=t_aero, thrust_prop=t_prop, rpm=sol["rpm"],
        J=sol["J"], eta_prop=sol["eta_prop"], eta_motor=mp["eta_motor"],
        eta_total=eta_total, current=mp["current"], voltage=mp["voltage"],
        p_shaft=sol["P_shaft"], p_elec=mp["p_elec"],
        feasible=(len(reasons) == 0), reasons=tuple(reasons))


# ---- Vollgas-Betriebspunkt (Drehmoment-Bilanz Motor = Propeller) --------
def _full_throttle_rpm(prop, motor, u_bus, v, rho):
    D = prop.diameter_m
    Kt = motor.Kt

    def motor_torque(rpm):
        I = (u_bus - rpm / motor.kv) / motor.Rm
        return (I - motor._i0_at(rpm)) * Kt

    def prop_torque(rpm):
        n = rpm / 60.0
        if n <= 0:
            return 0.0
        J = v / (n * D) if v > 1e-9 else 0.0
        return prop.CP(J, rpm) * rho * n * n * D ** 5 / (2 * math.pi)

    g = lambda rpm: motor_torque(rpm) - prop_torque(rpm)
    rpm_hi = motor.kv * u_bus * 0.9999
    if g(1.0) <= 0 or g(rpm_hi) >= 0:
        return None
    return brentq(g, 1.0, rpm_hi, xtol=0.5, maxiter=200)


def full_throttle_point(prop, motor, battery, v, rho):
    """Vollgas: liefert thrust, rpm, current, p_elec. None wenn nicht loesbar."""
    u_bus = battery.v_full          # Vollgas -> volle Spannung
    rpm = _full_throttle_rpm(prop, motor, u_bus, v, rho)
    if rpm is None:
        return None
    n = rpm / 60.0
    J = v / (n * prop.diameter_m) if v > 1e-9 else 0.0
    T = prop.CT(J, rpm) * rho * n * n * prop.diameter_m ** 4
    I = (u_bus - rpm / motor.kv) / motor.Rm
    return dict(thrust=max(T, 0.0), rpm=rpm, J=J, current=I,
                p_elec=u_bus * I)


def static_max_thrust(prop, motor, battery, rho):
    p = full_throttle_point(prop, motor, battery, 0.0, rho)
    return p["thrust"] if p else 0.0


def vmax(prop, motor, airframe, battery, pusher_factor=0.92, v_hi=80.0):
    """Geschwindigkeit, bei der Vollgas-Schub = Widerstand (drehzahllimitiert)."""
    rho = airframe.rho

    def excess(v):
        p = full_throttle_point(prop, motor, battery, v, rho)
        if p is None:
            return -1.0
        return p["thrust"] * pusher_factor - airframe.drag(v)

    lo = airframe.v_cruise
    if excess(lo) <= 0:
        return None                       # schafft nicht mal Cruise mit Vollgas
    if excess(v_hi) > 0:
        return v_hi                       # > Suchgrenze
    return brentq(excess, lo, v_hi, xtol=0.05, maxiter=200)


def endurance(p_elec_cruise, battery_wh, reserve=0.20, v_cruise=20.0):
    """Flugzeit [h] und Reichweite [km] aus el. Cruise-Leistung."""
    if p_elec_cruise <= 0 or not battery_wh:
        return None
    t_h = battery_wh * (1.0 - reserve) / p_elec_cruise
    return dict(time_h=t_h, range_km=t_h * v_cruise * 3.6)


def ideal_kv(cruise_rpm, u_bus):
    """Kv, das die Cruise-Drehzahl bei voller Spannung gerade liefert."""
    return cruise_rpm / u_bus if u_bus > 0 else 0.0
