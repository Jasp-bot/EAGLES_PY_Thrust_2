#!/usr/bin/env python3
"""
Manoever- und Kreisellasten auf die Motorbefestigung (Projekt EAGLES / STEVE).

Schaetzt die Momente an der Motorhalterung eines Pusher-Antriebs sowie die
resultierenden Schraubenkraefte fuer zwei Lastfaelle ab:

  A) Abfangbogen  : stationaerer Pull-up, gegeben Eintrittsgeschwindigkeit v
                    und erreichtes Lastvielfaches n -> konstante Nickrate
                    q = (n-1)*g/v.
  B) Sinus-Schwingung (Short-Period-Approx.): sinusfoermige Anstellwinkel-
                    aenderung mit Amplitude DELTA_ALPHA und Periode T_DELTA.
                    q_max = dalpha*(2*pi/T), qd_max = dalpha*(2*pi/T)^2.
                    Das Lastvielfache ist damit periodenabhaengig.

Achsen (x = Rotorachse, Symmetrieebene):
    x  Rotor-/Schubachse   (Schub erzeugt KEIN Nickmoment)
    y  Nickachse           -> M_y
    z  Gierachse           -> M_z

Physik:
  - Kreiselmoment (Rotor dreht um x, Flugzeug nickt um y -> Moment um z):
        M_z = I_p * omega * q
  - Nick-Traegheitsmoment des ueberhaengenden Motor-Rotor-Pakets:
        M_y = m * a_z * e_x  +  I_t * qd
        a_z = n*g (Abfangbogen)  bzw.  g + qd*l_SP (Sinus)
  - Rotor als Stabmodell:  I_p = ROD_COEFF * m_rotor * D^2   (ROD_COEFF=1/12)
        Quertraegheit      I_t = I_T_FACTOR * I_p            (~1/2)
  - Sicherheitsfaktor SF_RPM wirkt NUR auf die Drehzahl (omega).
  - Schrauben (N auf Lochkreisradius r, Ebene y-z, Normale = x):
        Zug je Schraube  = T/N + 2*M_res/(N*r)     (M_res = sqrt(M_y^2+M_z^2))
        Querkraft je Schr= F_z/N

Bauteildaten kommen aus der PyThrust-DB (wie run_bwb.py) ODER werden unten per
Override haendisch gesetzt (Masse/Abmasse). Ein Override (!= None) hat Vorrang.
"""

import sys
from datetime import datetime
from pathlib import Path
import math

import numpy as np

from bwb_propulsion_optimizer import (
    Filters, load_motors_dir, load_props_dir, Reporter, next_run_dir,
)

# --- Pfade relativ zu DIESER Datei -----------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent          # .../Calculations_EAGLES
REPO_ROOT  = SCRIPT_DIR.parent                         # .../PY_Thrust
DATA_DIR   = REPO_ROOT / "data"
MOTORS_DIR = DATA_DIR / "motors_T_Motor_pusher"
PROPS_DIR  = DATA_DIR / "propellers"

G = 9.80665
IN2M = 0.0254

# ========================= KONFIG ==========================================

# --- Bauteilquelle ---------------------------------------------------------
USE_DB        = True              # True: aus DB laden (Auswahl unten), dann Overrides anwenden
MOTOR_SELECT  = "AT4130-300"      # Name/ID-Teilstring fuer den Motor (DB)
PROP_SELECT   = "17x12E"          # Name/ID-Teilstring fuer den Propeller (DB)

# --- Manuelle Overrides (None = Wert aus DB nehmen) ------------------------
# Masse & Abmessungen lassen sich hier unabhaengig von der DB ueberschreiben.
M_MOTOR_G     = None              # Motormasse [g]            (DB: 405.4 fuer AT4130-300)
M_ROTOR_G     = 70.0             # Rotormasse [g]            (DB-Props haben oft keine Masse!)
D_PROP_IN     = None             # Propellerdurchmesser [in] (DB: 17.0)
MOTOR_LEN_MM  = 79.0             # Motorbaulaenge [mm]       (AT4130: 79 mm) -> fuer e_x-Default

# --- Geometrie -------------------------------------------------------------
L_SP_M        = (1300-632)/1000             # Abstand Flugzeug-SP -> Motorhalterung [m]  (BITTE ANPASSEN)
E_X_M         = 0.06             # axialer Ueberhang Schraubenebene -> Motor/Rotor-CG [m]
#                                  None -> Default ~ halbe Motorlaenge (MOTOR_LEN_MM/2)

# --- Schraubenbild (Stirnbefestigung, Ebene y-z) ---------------------------
N_BOLTS       = 4                # Anzahl Schrauben
R_BOLT_M      = 0.01           # Lochkreisradius [m] (z.B. 25 mm)

# --- Drehzahl / Kreisel ----------------------------------------------------
RPM_FULLTHROTTLE = 7000.0        # Vollgasdrehzahl Rotor [min^-1] (worst case)
SF_RPM           = 1.5           # Sicherheitsfaktor NUR auf die Drehzahl
ROD_COEFF        = 1.0 / 12.0    # I_p = ROD_COEFF * m_rotor * D^2 (Stab, 2-Blatt)
I_T_FACTOR       = 0.5           # I_t = I_T_FACTOR * I_p (Quertraegheit)

# --- Schub (axiale Schraubenlast, worst case) ------------------------------
THRUST_N      = 61.0             # max. Standschub [N] (worst case; SF wirkt NICHT auf Schub)

# --- Lastfall A: Abfangbogen ----------------------------------------------
PULLUP_ON     = True
PULLUP_V      = 40             # Eintrittsgeschwindigkeit in den Bogen [m/s]
PULLUP_N      = 3.0              # erreichtes Lastvielfaches [-]

# --- Lastfall B: Sinus-Anstellwinkelschwingung (Short-Period) -------------
SINUS_ON      = True
DELTA_ALPHA_DEG = 26           # Amplitude der Anstellwinkelaenderung [Grad]
T_DELTA_S       = 0.9    #1.2      # Periodendauer [s]
N_STEPS         = 720            # Aufloesung ueber eine Periode

# --- Lastfall C (optional): einzelner 1-cos-Nickpuls -----------------------
# Einmaliges Anstellen und Zurueck (Boe-aehnlich) statt Dauerschwingung;
# nutzt DELTA_ALPHA_DEG und T_DELTA_S. Standard aus.
PULSE_ON      = True

# --- Ausgabe ---------------------------------------------------------------
SAVE_RESULTS  = True             # versionierter Eintrag unter Results/
MAKE_PLOT     = True             # M_y(t)/M_z(t) der Schwingung plotten (matplotlib)
# ===========================================================================


# --------------------------------------------------------------------------
def _from_db():
    """Motor+Prop aus der DB holen (erste Treffer der Auswahl). None bei Fehlschlag."""
    if not MOTORS_DIR.is_dir() or not PROPS_DIR.is_dir():
        return None, None
    motors = load_motors_dir(str(MOTORS_DIR), filters=Filters(motor_select=MOTOR_SELECT))
    props  = load_props_dir(str(PROPS_DIR),  filters=Filters(prop_select=PROP_SELECT))
    m = motors[0] if motors else None
    p = props[0] if props else None
    return m, p


def resolve_components(out: Reporter):
    """Effektive Bauteildaten bestimmen: DB laden, dann Overrides anwenden."""
    motor = prop = None
    if USE_DB:
        motor, prop = _from_db()

    # Defaults aus DB (falls vorhanden)
    m_motor_g = motor.mass_kg * 1000.0 if motor else None
    m_rotor_g = prop.mass_kg * 1000.0 if prop else None
    d_prop_in = (prop.diameter_m / IN2M) if prop else None
    motor_name = motor.name if motor else "(manuell)"
    prop_name  = prop.name if prop else "(manuell)"

    # Overrides (haben Vorrang)
    if M_MOTOR_G is not None: m_motor_g = M_MOTOR_G
    if M_ROTOR_G is not None: m_rotor_g = M_ROTOR_G
    if D_PROP_IN is not None: d_prop_in = D_PROP_IN

    # Pflichtwerte pruefen
    missing = [n for n, v in (("Motormasse", m_motor_g), ("Rotormasse", m_rotor_g),
                              ("Prop-Durchmesser", d_prop_in)) if v in (None, 0.0)]
    if missing:
        out.print(f"[!] Fehlende Bauteildaten: {', '.join(missing)} "
                  f"-> bitte Override setzen oder DB-Auswahl pruefen.")
        sys.exit(1)

    e_x = E_X_M if E_X_M is not None else (MOTOR_LEN_MM / 1000.0) / 2.0

    return dict(
        motor_name=motor_name, prop_name=prop_name,
        m_motor=m_motor_g / 1000.0, m_rotor=m_rotor_g / 1000.0,
        d_prop=d_prop_in * IN2M, e_x=e_x,
        m_total=(m_motor_g + m_rotor_g) / 1000.0,
    )


def bolt_loads(M_res, F_z, thrust, n_bolts, r):
    """Schraubenkraefte: Zug (axial) und Querkraft (Schub) je Schraube."""
    f_tension = thrust / n_bolts + 2.0 * M_res / (n_bolts * r)
    f_shear   = F_z / n_bolts
    f_combined = math.hypot(f_tension, f_shear)
    return f_tension, f_shear, f_combined


# --------------------------------------------------------------------------
def main():
    out = Reporter()
    run_dir = version = None
    if SAVE_RESULTS:
        run_dir, version = next_run_dir(SCRIPT_DIR / "Results_Lasten", prefix="last")

    c = resolve_components(out)

    # Abgeleitete Groessen
    omega = SF_RPM * RPM_FULLTHROTTLE * 2.0 * math.pi / 60.0      # rad/s (mit SF)
    R = c["d_prop"] / 2.0
    I_p = ROD_COEFF * c["m_rotor"] * c["d_prop"] ** 2             # polare Traegheit (Stab)
    I_t = I_T_FACTOR * I_p                                        # Quertraegheit
    H = I_p * omega                                               # Drehimpuls

    # ---- Kopf / Konfiguration --------------------------------------------
    out.print(f"Lastberechnung Motorbefestigung"
              + (f"   (Lauf #{version:03d})" if version else ""))
    out.print(f"{datetime.now():%Y-%m-%d %H:%M}")
    out.blank()
    out.print("== Bauteile ==")
    out.print(f"  Motor / Rotor : {c['motor_name']} / {c['prop_name']}")
    out.print(f"  Masse Motor   : {c['m_motor']*1000:.1f} g")
    out.print(f"  Masse Rotor   : {c['m_rotor']*1000:.1f} g")
    out.print(f"  Masse gesamt  : {c['m_total']*1000:.1f} g  (Punktmasse fuer Manoever)")
    out.print(f"  D_prop        : {c['d_prop']/IN2M:.1f} in  = {c['d_prop']:.3f} m")
    out.blank()
    out.print("== Geometrie / Schrauben ==")
    out.print(f"  l_SP          : {L_SP_M:.3f} m  (SP -> Halterung)")
    out.print(f"  e_x (Ueberhang): {c['e_x']*1000:.1f} mm  (Schraubenebene -> CG)")
    out.print(f"  Schrauben     : {N_BOLTS} Stk auf r = {R_BOLT_M*1000:.1f} mm (Ebene y-z)")
    out.blank()
    out.print("== Drehzahl / Rotor-Traegheit ==")
    out.print(f"  rpm Vollgas   : {RPM_FULLTHROTTLE:.0f} min^-1")
    out.print(f"  SF (nur rpm)  : x{SF_RPM:.2f}  ->  {SF_RPM*RPM_FULLTHROTTLE:.0f} min^-1"
              f"  (omega = {omega:.0f} rad/s)")
    out.print(f"  I_p (Stab)    : {I_p:.4e} kg m^2   (ROD_COEFF={ROD_COEFF:.4f})")
    out.print(f"  I_t           : {I_t:.4e} kg m^2   (I_t = {I_T_FACTOR}*I_p)")
    out.print(f"  Drehimpuls H  : {H:.3f} kg m^2/s")
    out.print(f"  Schub (axial) : {THRUST_N:.1f} N  (worst case, ohne SF)")
    out.blank()

    worst = {"name": None, "M_res": 0.0, "tension": 0.0, "shear": 0.0,
             "M_y": 0.0, "M_z": 0.0}

    def consider(name, M_y, M_z, F_z):
        M_res = math.hypot(M_y, M_z)
        t, s, comb = bolt_loads(M_res, F_z, THRUST_N, N_BOLTS, R_BOLT_M)
        out.print(f"  M_y (Nick, Traegheit) : {M_y:8.2f} N m")
        out.print(f"  M_z (Gier, Kreisel)   : {M_z:8.2f} N m")
        out.print(f"  M_res = sqrt(My^2+Mz^2): {M_res:8.2f} N m")
        out.print(f"  Normalkraft F_z       : {F_z:8.2f} N")
        out.print(f"  -> Schraube Zug (max) : {t:8.2f} N  (kritische Schraube)")
        out.print(f"  -> Schraube Quer      : {s:8.2f} N")
        out.print(f"  -> Schraube kombiniert: {comb:8.2f} N")
        if M_res > worst["M_res"]:
            worst.update(name=name, M_res=M_res, tension=t, shear=s,
                         M_y=M_y, M_z=M_z)

    # ---- Lastfall A: Abfangbogen -----------------------------------------
    if PULLUP_ON:
        q = (PULLUP_N - 1.0) * G / PULLUP_V          # konstante Nickrate [rad/s]
        a_z = PULLUP_N * G                           # Normalbeschleunigung
        M_y = c["m_total"] * a_z * c["e_x"]          # qd = 0 -> nur Massenterm
        M_z = I_p * omega * q                        # Kreisel
        F_z = c["m_total"] * a_z
        out.print(f"== Lastfall A: Abfangbogen  (v={PULLUP_V:.0f} m/s, n={PULLUP_N:.1f}) ==")
        out.print(f"  Nickrate q   : {q:.4f} rad/s  ({math.degrees(q):.1f} deg/s), qd = 0")
        consider("Abfangbogen", M_y, M_z, F_z)
        out.blank()

    # ---- Lastfall B: Sinus-Schwingung ------------------------------------
    My_t = Mz_t = t_arr = None
    if SINUS_ON:
        dalpha = math.radians(DELTA_ALPHA_DEG)
        w = 2.0 * math.pi / T_DELTA_S
        t_arr = np.linspace(0.0, T_DELTA_S, N_STEPS)
        q_t  =  dalpha * w * np.cos(w * t_arr)       # Nickrate
        qd_t = -dalpha * w * w * np.sin(w * t_arr)   # Nickbeschleunigung
        a_z_t = G + qd_t * L_SP_M                    # 1g immer + kinematisch
        My_t = c["m_total"] * a_z_t * c["e_x"] + I_t * qd_t
        Mz_t = I_p * omega * q_t
        Mres_t = np.hypot(My_t, Mz_t)
        Fz_t = c["m_total"] * a_z_t
        k = int(np.argmax(Mres_t))                   # Phase mit groesstem Resultierenden

        out.print(f"== Lastfall B: Anstellwinkelschwingung  "
                  f"(dalpha={DELTA_ALPHA_DEG:.1f} deg, T={T_DELTA_S:.2f} s) ==")
        out.print(f"  q_max  : {dalpha*w:.4f} rad/s  ({math.degrees(dalpha*w):.1f} deg/s)")
        out.print(f"  qd_max : {dalpha*w*w:.4f} rad/s^2")
        out.print(f"  Lastvielfaches n_max ~ {1.0 + abs(qd_t).max()*L_SP_M/G:.2f} "
                  f"(periodenabhaengig, kinematisch)")
        out.print(f"  -- Maxima ueber eine Periode --")
        out.print(f"  |M_y|_max : {np.abs(My_t).max():8.2f} N m")
        out.print(f"  |M_z|_max : {np.abs(Mz_t).max():8.2f} N m")
        out.print(f"  -- kritische Phase (max. M_res, t={t_arr[k]:.3f} s) --")
        consider("Schwingung", float(My_t[k]), float(Mz_t[k]), float(Fz_t[k]))
        out.blank()

    # ---- Lastfall C: einzelner 1-cos-Nickpuls (optional) -----------------
    if PULSE_ON:
        dalpha = math.radians(DELTA_ALPHA_DEG)
        w = 2.0 * math.pi / T_DELTA_S
        tp = np.linspace(0.0, T_DELTA_S, N_STEPS)
        # alpha(t) = (dalpha/2)*(1-cos(w t))  -> einmaliges Anstellen und zurueck
        q_p  = (dalpha / 2.0) * w * np.sin(w * tp)
        qd_p = (dalpha / 2.0) * w * w * np.cos(w * tp)
        a_z_p = G + qd_p * L_SP_M
        My_p = c["m_total"] * a_z_p * c["e_x"] + I_t * qd_p
        Mz_p = I_p * omega * q_p
        Mres_p = np.hypot(My_p, Mz_p)
        Fz_p = c["m_total"] * a_z_p
        kp = int(np.argmax(Mres_p))
        out.print(f"== Lastfall C: 1-cos-Nickpuls  "
                  f"(dalpha={DELTA_ALPHA_DEG:.1f} deg, T={T_DELTA_S:.2f} s) ==")
        out.print(f"  qd_max : {(dalpha/2.0)*w*w:.4f} rad/s^2  (halbe Schwingungsamplitude)")
        out.print(f"  -- kritische Phase (t={tp[kp]:.3f} s) --")
        consider("1-cos-Puls", float(My_p[kp]), float(Mz_p[kp]), float(Fz_p[kp]))
        out.blank()

    # ---- Gesamt-Worst-Case -----------------------------------------------
    out.print("== AUSLEGUNG (Worst Case ueber alle Lastfaelle) ==")
    out.print(f"  massgebender Lastfall : {worst['name']}")
    out.print(f"  M_y / M_z             : {worst['M_y']:.2f} / {worst['M_z']:.2f} N m")
    out.print(f"  M_res                 : {worst['M_res']:.2f} N m")
    out.print(f"  Schraubenzug  (max)   : {worst['tension']:.2f} N")
    out.print(f"  Schraubenquer (max)   : {worst['shear']:.2f} N")
    out.blank()
    out.print("Hinweise: SF wirkt nur auf die Drehzahl (Kreisel). Schub erzeugt kein "
              "Nickmoment (x=Symmetrieebene); er geht nur axial in die Schrauben. "
              "Rotor als Stabmodell (I_p=m*D^2/12). M_z=Kreisel (Gier), M_y=Traegheit (Nick).")

    # ---- Plot + Speichern ------------------------------------------------
    if SAVE_RESULTS and run_dir is not None:
        if MAKE_PLOT and My_t is not None:
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, ax = plt.subplots(figsize=(8, 4.5))
                ax.plot(t_arr, My_t, label="M_y (Nick, Traegheit)", color="#2E5C8A")
                ax.plot(t_arr, Mz_t, label="M_z (Gier, Kreisel)", color="#C0392B")
                ax.plot(t_arr, np.hypot(My_t, Mz_t), "--", color="#555555",
                        label="M_res")
                ax.set_xlabel("Zeit t [s]"); ax.set_ylabel("Moment [N m]")
                ax.set_title("Momente an der Motorbefestigung — Anstellwinkelschwingung")
                ax.grid(alpha=0.3); ax.legend()
                fig.tight_layout()
                fig.savefig(run_dir / "momente_schwingung.png", dpi=130)
                plt.close(fig)
            except Exception as e:        # Plot ist optional
                print(f"[Plot uebersprungen] {e}")
        out.save(run_dir / "lasten.txt",
                 title=f"Motorbefestigung - Lastberechnung (Lauf {version:03d})")
        print(f"\nErgebnisse gespeichert in: {run_dir}")


if __name__ == "__main__":
    main()
