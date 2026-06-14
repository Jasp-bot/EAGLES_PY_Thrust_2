#!/usr/bin/env python3
"""
Beispiellauf BWB-Pusher.

Standard: liest die PyThrust-Datenbank aus dem data/-Ordner des Repos
(relativ zu DIESER Datei aufgeloest, daher unabhaengig vom PyCharm-
Arbeitsverzeichnis). Erwartete Lage:

    PY_Thrust/
      Calculations_EAGLES/ bwb_propulsion_optimizer/ , run_bwb.py
      data/ motors/*.json , propellers/**/*.json (+ *.csv)

Aufrufe:
    python run_bwb.py                       # data/ aus dem Repo (Default)
    python run_bwb.py <motor-ordner> <prop-ordner>   # Pfade ueberschreiben
    python run_bwb.py --demo                # synthetische Beispieldaten

Filter unten im KONFIG-Block: Kv-Bereich, Durchmesser-/Steigungs-/P/D-Bereich,
Blattzahl, Motormasse-Limit, HERSTELLER (mehrere gleichzeitig zum Vergleich)
und gezielte EINZELAUSWAHL von Motoren/Propellern (Name/ID-Teilstring).
"""
import sys
from datetime import datetime
from pathlib import Path
from bwb_propulsion_optimizer import (
    Airframe, Battery, Filters, optimize, format_table,
    example_motors, example_props, load_motors_dir, load_props_dir,
    Reporter, next_run_dir,
)

# --- Pfade relativ zu DIESER Datei (robust gegen das Arbeitsverzeichnis) ---
SCRIPT_DIR = Path(__file__).resolve().parent          # .../Calculations_EAGLES
REPO_ROOT  = SCRIPT_DIR.parent                         # .../PY_Thrust
DATA_DIR   = REPO_ROOT / "data"
MOTORS_DIR = DATA_DIR / "motors_T_Motor_pusher"
PROPS_DIR  = DATA_DIR / "propellers"                   # rekursiv (inkl. Unterordner)

# ========================= KONFIG ==========================================
MASS_KG       = 17.0          # Abflugmasse des BWB-Pushers (inkl. Akku, Motor, Prop, ...), kg
V_CRUISE      = 20.0        # Reisegeschwindigkeit, m/s (ca. 72 km/h, typisch für kleine E-Flugzeuge)
V_MAX_TARGET  = 30.0        # maximal erreichbare Fluggeschwindigkeit
GLIDE_RATIO   = 10.0          # L/D im Reiseflug (Endurance haengt hieran)
PUSHER_FACTOR = 0.92
DESIGN_THRUST = 22.0          # konservativer Reserve-/Auslegungsschub (Start/Steig)
BATTERY_WH    = 816        # ~ >30 Ah @ 12S; fuer Flugzeit/Reichweite
RESERVE       = 0.20


# --- Bordnetz / Zellzahl (frei waehlbar, NICHT an 12S gebunden) -------------
CELLS          = 6          # Zellenzahl (S): z.B. 6, 8, 10, 12, 14
V_CELL_FULL    = 4.20        # Zellspannung voll  [V] (LiPo-Default)
V_CELL_NOMINAL = 3.70        # Zellspannung nominal[V]
V_CELL_EMPTY   = 3.50        # Zellspannung leer  [V] (Cruise-Machbarkeit)

# --- Vmax-Fenster: Kombi MUSS v_max erreichen, aber max. v_max+Cap ----------
VMAX_REACH        = True     # Vmax >= v_max erforderlich (erreichen koennen)
VMAX_CAP_MARGIN   = 10.0     # Obergrenze Loesungsraum = v_max + 10 (None = aus)

# --- Weicher Mindest-Lastpunkt im Cruise (gegen 10%-Throttle) ---------------
LOAD_MIN            = 0.30   # Ziel-Mindestlastpunkt (30 %)
LOAD_TOL            = 0.05   # weicher Uebergang +-5 %
LOAD_PENALTY_WEIGHT = 0.5    # Staerke der Strafe im Score (0 = aus)

# --- Antriebsmasse: NUR Anzeige + Tiebreak bei quasi gleichem Score ---------
MASS_TIEBREAK     = True
# --- Filter: Bereiche (None = keine Grenze) --------------------------------
KV_MIN        = 100.0         # Motoren: Kv-Bereich [rpm/V]
KV_MAX        = 500.0
DMIN_IN       = 12.0          # Propeller: Durchmesserbereich [Zoll]
DMAX_IN       = 20.0
PMIN_IN       = None          # Propeller: Steigungsbereich [Zoll]
PMAX_IN       = 12
PD_MIN        = None          # Propeller: P/D-Verhaeltnis (z.B. 0.55 .. 0.75)
PD_MAX        = None
BLADES        = 2          # erlaubte Blattzahlen, z.B. [2] oder [2, 3]
MOTOR_MASS_MAX_G = None       # Motoren: Gewichtslimit [g], z.B. 500

# --- Filter: Hersteller (Listen -> mehrere gleichzeitig vergleichen) -------
# Beispiel Vergleich:  MOTOR_MFR = ["T-Motor", "HobbyWing"]
MOTOR_MFR     = None #["Hobbywing"]     # None = alle Hersteller
PROP_MFR      = None          # z.B. ["APC", "Aeronaut"]

# --- Filter: gezielte Einzelauswahl (Name/ID-Teilstring, Liste) ------------
# Beispiel manueller Abgleich:  MOTOR_SELECT = ["U8 II", "AT4120"]
#                               PROP_SELECT  = ["19x13", "20x13E"]
MOTOR_SELECT  = None #["AT4125-250","AX435-B-220", "C6225-200", "C6220-220"]          # None = alle Motoren
PROP_SELECT   = None          # None = alle Propeller

# --- Grafische Auswertung --------------------------------------------------
MAKE_PLOTS    = True          # Plots der Top 5 erzeugen (braucht matplotlib)
SHOW_PLOTS    = False        # zusaetzlich Fenster oeffnen (plt.show())

# Laufzeit: teure Groessen (Vmax/Standschub/Startschub/Endurance) nur fuer die
# besten K Kombinationen. None = fuer alle (volles Vmax-Streudiagramm, langsamer).
HEAVY_METRICS_TOP = 10
# ===========================================================================


def build_filters() -> Filters:
    return Filters(
        kv_min=KV_MIN, kv_max=KV_MAX,
        dmin_in=DMIN_IN, dmax_in=DMAX_IN,
        pmin_in=PMIN_IN, pmax_in=PMAX_IN, pd_min=PD_MIN, pd_max=PD_MAX,
        blades=BLADES, motor_mass_max_g=MOTOR_MASS_MAX_G,
        motor_mfr=MOTOR_MFR, prop_mfr=PROP_MFR,
        motor_select=MOTOR_SELECT, prop_select=PROP_SELECT)


def _load(motors_dir: Path, props_dir: Path, filt: Filters):
    if not motors_dir.is_dir() or not props_dir.is_dir():
        print(f"[!] Daten-Ordner nicht gefunden:\n    {motors_dir}\n    {props_dir}")
        return None
    # Filter schon beim Laden: ausgeschlossene Props parsen keine CSV (Ladezeit!)
    motors = load_motors_dir(str(motors_dir), filters=filt)
    props = load_props_dir(str(props_dir), filters=filt)
    if not motors or not props:
        print(f"[!] Ordner gefunden, aber nach Filter leer "
              f"({len(motors)} Motoren, {len(props)} Props).")
        return None
    return motors, props


def main():
    filt = build_filters()
    out = Reporter()
    run_dir, version = next_run_dir(SCRIPT_DIR / "Results")

    if "--demo" in sys.argv:
        motors, props = example_motors(), example_props()
        from bwb_propulsion_optimizer import filter_components
        motors, props = filter_components(motors, props, filt)
        src = "SYNTHETISCHE Beispieldaten (--demo)"
    elif len(sys.argv) == 3:
        loaded = _load(Path(sys.argv[1]), Path(sys.argv[2]), filt)
        motors, props = loaded if loaded else (example_motors(), example_props())
        src = (f"PyThrust-DB: {sys.argv[1]} / {sys.argv[2]}"
               if loaded else "Fallback: SYNTHETISCHE Beispieldaten")
    else:
        loaded = _load(MOTORS_DIR, PROPS_DIR, filt)
        if loaded:
            motors, props = loaded
            src = f"PyThrust-DB (Repo): {MOTORS_DIR} / {PROPS_DIR}"
        else:
            motors, props = example_motors(), example_props()
            src = "Fallback: SYNTHETISCHE Beispieldaten (data/ nicht nutzbar)"

    af = Airframe(mass_kg=MASS_KG, v_cruise=V_CRUISE,
                  glide_ratio=GLIDE_RATIO, cd0_fraction=0.5)
    bat = Battery(cells=CELLS, v_cell_full=V_CELL_FULL, v_cell_nominal=V_CELL_NOMINAL,
                  v_cell_empty=V_CELL_EMPTY, design_state="empty")

    # ===== Kopf: Lauf-Nr. + Konfiguration (kommt so auch in results.txt) =====
    out.print(f"Lauf #{version:03d}    {datetime.now():%Y-%m-%d %H:%M}")
    out.blank()
    out.print("== Konfiguration ==")
    out.print(f"  Flugzeug : MASS_KG={MASS_KG}  V_CRUISE={V_CRUISE}  "
              f"V_MAX_TARGET={V_MAX_TARGET}  GLIDE_RATIO={GLIDE_RATIO}  "
              f"PUSHER_FACTOR={PUSHER_FACTOR}")
    out.print(f"  Schub/Akku: DESIGN_THRUST={DESIGN_THRUST}  BATTERY_WH={BATTERY_WH}  "
              f"RESERVE={RESERVE}")
    out.print(f"  Bordnetz : CELLS={CELLS}  "
              f"V_CELL={V_CELL_FULL}/{V_CELL_NOMINAL}/{V_CELL_EMPTY}")
    out.print(f"  Filter   : {filt.summary()}")
    out.print(f"  Vmax     : REACH={VMAX_REACH}  CAP_MARGIN={VMAX_CAP_MARGIN}")
    out.print(f"  Lastpunkt: MIN={LOAD_MIN}  TOL={LOAD_TOL}  WEIGHT={LOAD_PENALTY_WEIGHT}")
    out.print(f"  Sonstiges: MASS_TIEBREAK={MASS_TIEBREAK}  HEAVY_METRICS_TOP={HEAVY_METRICS_TOP}")
    out.print(f"  Pfade    : {MOTORS_DIR}  |  {PROPS_DIR}")
    out.blank()

    out.print("== Lauf ==")
    out.print(f"Quelle      : {src}")
    out.print(f"Filter      : {filt.summary()}")
    out.print(f"Bestand     : {len(motors)} Motoren x {len(props)} Propeller "
              f"= {len(motors)*len(props)} Kombinationen")
    if props:
        d_in = sorted(p.diameter_m / 0.0254 for p in props)
        mfrs = sorted({(p.manufacturer or '?') for p in props})
        out.print(f'Prop-D      : {d_in[0]:.1f}\"..{d_in[-1]:.1f}\" | '
                  f"Hersteller: {', '.join(mfrs)[:60]}")
    if motors:
        mm = sorted({(m.manufacturer or '?') for m in motors})
        out.print(f"Motoren     : Hersteller: {', '.join(mm)[:60]}")
    out.print(f"Flugzeug    : {MASS_KG:.0f} kg, L/D={GLIDE_RATIO:.0f}, "
              f"v_cruise={V_CRUISE:.0f}, v_max-Ziel={V_MAX_TARGET:.0f} m/s")
    out.print(f"Schub real  : {af.thrust_required(V_CRUISE):.1f} N @cruise "
              f"(= echter aerodyn. Widerstand aus L/D)")
    out.print(f"Bordnetz    : {CELLS}S (U_voll={bat.v_full:.0f} / "
              f"U_design={bat.v_design:.0f} V)")
    _cap = f"..{V_MAX_TARGET+VMAX_CAP_MARGIN:.0f}" if VMAX_CAP_MARGIN else ""
    out.print(f"Vmax-Fenster: {V_MAX_TARGET:.0f}{_cap} m/s  |  "
              f"Mindest-Lastpunkt {LOAD_MIN*100:.0f}% (+-{LOAD_TOL*100:.0f}%, "
              f"w={LOAD_PENALTY_WEIGHT})")
    out.print(f"Akku        : {BATTERY_WH:.0f} Wh, Reserve {RESERVE*100:.0f}%")
    out.blank()

    cands_all = optimize(motors, props, af, bat,
                         v_max_target=V_MAX_TARGET, pusher_factor=PUSHER_FACTOR,
                         filters=filt, design_thrust=DESIGN_THRUST,
                         battery_wh=BATTERY_WH, reserve=RESERVE,
                         vmax_reach=VMAX_REACH, vmax_cap_margin=VMAX_CAP_MARGIN,
                         load_min=LOAD_MIN, load_tol=LOAD_TOL,
                         load_penalty_weight=LOAD_PENALTY_WEIGHT,
                         mass_tiebreak=MASS_TIEBREAK,
                         heavy_metrics_top=HEAVY_METRICS_TOP,
                         progress=True, top_n=None)   # alle -> Streudiagramme
    cands = cands_all[:10]

    out.print("Ranking nach el. Cruise-Leistung (kleiner = effizienter):")
    out.blank()
    out.print(format_table(cands))

    if cands:
        c = cands[0]
        out.blank()
        out.print("--- Top-Kombination im Detail "
                  "------------------------------------")
        out.print(f"  {c.motor.name} ({c.motor.manufacturer}) + "
                  f"{c.prop.name} ({c.prop.manufacturer})")
        out.print(f"  Mass   : {c.mass_total*1000:.0f} g (Motor+Rotor)")
        out.print(f"  Cruise : {c.cruise.thrust_aero:.1f} N @ {c.cruise.rpm:.0f} rpm, "
                  f"J={c.cruise.J:.2f}, eta_prop={c.cruise.eta_prop:.2f}, "
                  f"eta_ges={c.cruise.eta_total:.2f}")
        out.print(f"  Strom  : {c.cruise.current:.1f} A @{CELLS}S")
        out.print(f"  Last   : Cruise {c.load_point*100:.0f}% von "
                  f"{c.static_thrust:.0f} N Standschub (Ziel >= {LOAD_MIN*100:.0f}%)")
        if c.vmax:
            out.print(f"  Kv     : Motor {c.motor.kv:.0f}, ideal ~{c.kv_ideal:.0f} "
                      f"(Vmax {c.vmax:.0f} m/s)")
        if c.bungee_thrust:
            out.print(f"  Start  : {c.static_thrust:.0f} N statisch, "
                      f"{c.bungee_thrust:.0f} N @14 m/s (Bungee-Exit)")
        if c.endurance:
            out.print(f"  Flugzeit ~{c.endurance['time_h']:.1f} h, "
                      f"Reichweite ~{c.endurance['range_km']:.0f} km "
                      f"(bei L/D={GLIDE_RATIO:.0f}, {BATTERY_WH:.0f} Wh, "
                      f"{RESERVE*100:.0f}% Reserve)")
    out.blank()
    out.print("Spalten: \u03b7ges=Gesamtwirkungsgrad  P_cr=el.Leistung[W]  "
              "I=Strom[A]  Vmax[m/s]  Last=Lastpunkt  Masse=Motor+Rotor  "
              "Kv/Kvid=ist/ideal  Tstat=Standschub[N]")

    # --- Plots in den versionierten Run-Ordner -----------------------------
    if MAKE_PLOTS and cands_all:
        try:
            from bwb_propulsion_optimizer import make_plots
            paths = make_plots(cands_all, af, bat, pusher_factor=PUSHER_FACTOR,
                               v_max_target=V_MAX_TARGET,
                               dmin_in=(DMIN_IN or 15), dmax_in=(DMAX_IN or 20),
                               top=5, outdir=str(run_dir), show=SHOW_PLOTS)
            out.blank()
            out.print(f"Plots ({len(paths)}) im Run-Ordner gespeichert.")
        except ImportError as e:
            out.print(f"[Plots uebersprungen] {e}")

    # --- results.txt schreiben ---------------------------------------------
    txt = out.save(run_dir / "results.txt",
                   title=f"BWB Propulsion Optimizer - Lauf {version:03d}")
    print(f"\nErgebnisse gespeichert in: {run_dir}")
    print(f"  - results.txt  (Konfiguration + Konsole, ohne Plot-Status)")

if __name__ == "__main__":
    main()
