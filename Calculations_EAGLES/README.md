# BWB Propulsion Optimizer (v0.2)

Auswahl effizienter **Motor-/Propeller-Kombinationen** für ein Blended-Wing-Body
UAV in **Pusher-Konfiguration**. Architektur nach [PyThrust](https://github.com/Setuav/PyThrust)
(Beiwert-Solver CT(J)/CP(J) + Datenbanksuche). Die Auslegungslogik ist um die
Erkenntnisse aus dem STEVE-Antriebsverlauf erweitert.

## Grafische Auswertung der Top 5

Nach dem Ranking erzeugt das Tool fünf Abbildungen (matplotlib) und legt sie in
`Calculations_EAGLES/plots/` ab. In `run_bwb.py` steuerbar über `MAKE_PLOTS`
(an/aus) und `SHOW_PLOTS` (zusätzlich Fenster öffnen):

1. **`1_schub_vs_widerstand.png`** — Vollgas-Schub jeder Top-Kombi gegen den
   Widerstand über der Geschwindigkeit; Vmax = Schnittpunkt (Punkt markiert),
   Startüberschuss als Abstand bei niedrigem V. Marker für Cruise/Bungee/Vmax-Ziel.
2. **`2_strahltheorie.png`** — idealer Froude-Wirkungsgrad und Strahl-Austritts-
   geschwindigkeit über dem Durchmesser (warum großer Durchmesser effizienter ist).
3. **`3_auswahl_streudiagramme.png`** — η_prop vs. Kv-Abweichung (mit „gutem"
   Kv-Band) und η_prop vs. Vmax, eingefärbt nach Durchmesser, Top 5 als Sterne.
4. **`4_eta_landkarte.png`** — Cruise-η-Landkarte über Durchmesser × Steigung
   (Schätzer, zur Orientierung) mit P/D-Linien und den Top 5 als Sterne.
5. **`5_top_vergleich.png`** — Balkenvergleich der Top 5: el. Leistung, η_ges,
   Motor-Lastpunkt (40–70 %-Band), Vmax.

Programmatisch:

```python
from bwb_propulsion_optimizer import optimize, make_plots
cands = optimize(motors, props, af, bat, top_n=None)   # alle (Hintergrund)
make_plots(cands, af, bat, top=5, outdir="plots", show=True)
```

Benötigt `matplotlib` (`pip install matplotlib`; PyThrust-„plot"-Extra). Fehlt es,
läuft das Ranking normal weiter und nur die Plots werden übersprungen.

## Laufzeit eingrenzen (Filter) + Fortschrittsanzeige

In `run_bwb.py` lässt sich der Suchraum umfangreich einstellen
(alle Kriterien `None` = keine Einschränkung):

```python
KV_MIN, KV_MAX   = 100.0, 300.0   # Motor-Kv-Bereich [rpm/V]
DMIN_IN, DMAX_IN = 15.0, 20.0     # Prop-Durchmesser [Zoll]
PMIN_IN, PMAX_IN = None, None     # Prop-Steigung [Zoll]
PD_MIN, PD_MAX   = None, None     # P/D-Verhältnis (z.B. 0.55 .. 0.75)
BLADES           = None           # erlaubte Blattzahlen, z.B. [2] oder [2, 3]
MOTOR_MASS_MAX_G = None           # Motor-Gewichtslimit [g], z.B. 500
MOTOR_MFR = None                  # Hersteller, mehrere: ["T-Motor", "Scorpion"]
PROP_MFR  = None                  # z.B. ["APC", "Aeronaut"]
MOTOR_SELECT = None               # Einzelauswahl (Teilstring): ["U8 II", "AT4120"]
PROP_SELECT  = None               # z.B. ["19x13", "20x13E"]
```

Typische Anwendungen:
- **Zwei Hersteller vergleichen**: `MOTOR_MFR = ["T-Motor", "Scorpion"]` lädt
  beide und stellt sie im selben Ranking/Streudiagramm gegenüber.
- **Manueller Abgleich**: `MOTOR_SELECT = ["U8 II"]`, `PROP_SELECT = ["19x13"]`
  lassen den Optimizer nur über diese Kombination(en) laufen.
- **Budget/Geometrie**: `MOTOR_MASS_MAX_G = 500`, `PD_MIN, PD_MAX = 0.6, 0.75`,
  `BLADES = [2]`.

Die Filter greifen **schon beim Laden** (Hersteller/Auswahl/Durchmesser/Steigung
aus dem JSON, *bevor* die teure Performance-CSV geparst wird) — der größte
Laufzeit-Hebel bei einem großen APC-Katalog. Programmatisch über `Filters`:

```python
from bwb_propulsion_optimizer import Filters, load_motors_dir, load_props_dir, optimize
f = Filters(kv_min=100, kv_max=300, dmin_in=15, dmax_in=20,
            motor_mfr=["T-Motor", "Scorpion"], blades=[2], pd_min=0.6, pd_max=0.75)
motors = load_motors_dir("data/motors", filters=f)
props  = load_props_dir("data/propellers", filters=f)
cands  = optimize(motors, props, af, bat, filters=f)
```

**Schnellmodus bei sehr großen Katalogen** (`HEAVY_METRICS_TOP` in `run_bwb.py`):
Die billige Cruise-Bewertung (= Ranking) läuft für *alle* Kombinationen; die
teuren Größen (Vmax, Standschub, Startschub, Endurance) werden nur für die
besten K gerechnet. Das Ranking und die Top-Plots bleiben identisch, nur das
Vmax-Streudiagramm zeigt dann K statt aller Punkte. Beispiel: 44.421
Kombinationen 8,4 s → 1,4 s (≈6×). `None` = für alle rechnen (volles
Vmax-Streudiagramm, langsamer).

Programmgesteuert:

```python
from bwb_propulsion_optimizer import load_motors_dir, load_props_dir, optimize
motors = load_motors_dir("data/motors", kv_min=100, kv_max=300)
props  = load_props_dir("data/propellers", dmin_in=15, dmax_in=20)
cands  = optimize(motors, props, af, bat,
                  kv_min=100, kv_max=300, dmin_in=15, dmax_in=20,  # Sicherheitsnetz
                  progress=True)        # Fortschrittsanzeige an/aus
```

Während des Durchlaufs erscheint eine **Fortschrittsanzeige**
(`Optimiere [######----] 52% (685/1320) 0.2s`), abhängigkeitsfrei und für die
PyCharm-Run-Konsole geeignet. `progress=False` schaltet sie ab.

## Schnellstart

In PyCharm einfach `run_bwb.py` ausführen (▶). Die PyThrust-Datenbank wird
standardmäßig aus dem `data/`-Ordner des Repos gelesen — der Pfad wird relativ
zur Skriptdatei aufgelöst, also unabhängig vom Arbeitsverzeichnis. Erwartete Lage:

```
PY_Thrust/
  Calculations_EAGLES/bwb_propulsion_optimizer/ , run_bwb.py
  data/motors/*.json , data/propellers/**/*.json (+ *.csv)
```

```bash
python run_bwb.py                       # data/ aus dem Repo (Default)
python run_bwb.py <motor-ordner> <prop-ordner>   # Pfade ueberschreiben
python run_bwb.py --demo                # synthetische Beispieldaten
```

Abhängigkeiten: `numpy`, `scipy` (wie PyThrust).

## Was v0.2 neu kann

**1. Echtes PyThrust-Datenformat wird direkt geladen.** Aus den hochgeladenen
Beispieldateien ist das Schema jetzt bekannt:

- *Motor* = JSON: `kv`, `resistance`→Rm, `io`→I0, `io_voltage`, `max_current`→i_max,
  `max_power`→p_max, `weight_g`→Masse, `manufacturer`.
- *Propeller* = JSON (`diameter_in`, `pitch_in`, `blade_count`, `data_csv`) +
  verlinkte **APC-Performance-CSV** (`rpm, advance_ratio, thrust_coeff,
  power_coeff, efficiency, …`).

```python
from bwb_propulsion_optimizer import load_motors_dir, load_props_dir
motors = load_motors_dir("PyThrust/data/motors")   # alle *.json
props  = load_props_dir("PyThrust/data/props")      # JSON + CSV automatisch
```
Die CSV-Auflösung erkennt auch die Upload-Umbenennung (`APC_4.1x4.1E.csv` ↔
`APC_4_1x4_1E.csv`).

**2. Messdaten-Propeller statt Schätzer.** Liegt eine Performance-CSV vor, werden
**gemessene** CT(J)/CP(J) (RPM-/Reynolds-abhängig interpoliert) benutzt — der
„belastbare Weg", den der STEVE-Verlauf als Fazit zieht. Ohne CSV greift weiter
der grobe Glockenkurven-Schätzer (klar als Näherung markiert).

**3. Auslegungs-Entscheidungsgrößen aus dem STEVE-Verlauf**, pro Kombination:

| Größe | Bedeutung / STEVE-Bezug |
|------|--------------------------|
| **η_ges, P_cruise** | Reiseflug-Effizienz am **echten** aerodyn. Widerstand (L/D) |
| **Motor-Lastpunkt** | Auslegungsschub / Vollgas-Standschub; Ideal **40–70 %** (C6220-Lektion: nicht 10–15 % Teillast, nicht >90 %) |
| **Vmax** | drehzahl-/spannungslimitiert über `n≈Kv·U` — Vmax folgt dem **Kv** |
| **Kv ideal vs. ist** | ideales Kv aus Cruise-Drehzahl & Bordspannung |
| **Startschub** | statisch + am Bungee-Exit (~14 m/s) |
| **Strom @12S** | inkl. 6S-Vergleich (4× I²R — wichtig bei schlechten Lötstellen) |
| **Endurance** | Flugzeit/Reichweite aus Akku-Wh, Reserve, **echtem** L/D |

**Methodische Trennung** (Kernpunkt des Verlaufs): Effizienz & Endurance gegen den
realen aerodynamischen Widerstand (L/D), Reserve (Start, Vmax) gegen die
Vollgas-Fähigkeit. `design_thrust` setzt den konservativen Bezug für den Lastpunkt.

## Modellgleichungen (unverändert gültig)

Zelle: `T_cruise = m·g/(L/D)`, Widerstand über v in parasitär (∝v²) + induziert (∝1/v²).
Propeller: `T=CT·ρ·n²·D⁴`, `P=CP·ρ·n³·D⁵`, `η=J·CT/CP`; Drehzahl per `brentq`.
Motor: `Kt=60/(2π·Kv)`, `I=Q/Kt+I0`, `U=rpm/Kv+I·Rm`, `η=P_welle/P_el`.
Vollgas/Vmax: Drehmomentbilanz Motor=Propeller bei voller Spannung.

## Eckdaten (in `run_bwb.py`)

Masse 15–17 kg (16), v_cruise 20 m/s, v_max-Ziel 30 m/s, Gleitzahl 9–12 (10),
D_max 20″, 12S. Endurance gegen L/D; `DESIGN_THRUST` (Reserve) für den Lastpunkt.

## Wichtige Erkenntnisse aus dem STEVE-Verlauf (im Tool abgebildet)

- Großer Durchmesser / niedrige Scheibenbelastung → höherer Froude-Wirkungsgrad;
  Single-Prop schlägt zwei kleine. Strahlaustritt nur knapp über v_cruise halten.
- Reiseflug-J liegt bei flachen Props oft **links** vom η-Optimum; P/D ≈ 0,6–0,7
  schiebt J_opt ins Betriebsfenster (gemessene CT/CP fangen das automatisch ein).
- Vmax wird durch Kv begrenzt, nicht durch den Gashebel: zu hohes Kv → überschüssige
  Vmax + schlechterer Cruise-η. Für 28–30 m/s ist Kv ≈ 150–170 (12S) sinnvoll.
- 12S statt 6S: halber Strom, ¼ I²R — relevant bei langen Leitungen/Lötstellen.
- Reale Schubmessungen (z. B. AT5220A: Ct0≈0,083, Cp0≈0,033) schlagen reine
  Theorie — daher die Messdaten-Propeller-Pfad.
- Markt-Lücke: 12S-Fixed-Wing-Motor mit Kv≈120–170 **und** ~80 N ist selten
  (Kandidaten im Verlauf: U8 II KV150, MN605-S KV170, AX435, AT5230A KV200,
  C7225 KV160). Long-Shaft/Dreifachlager wegen **gyroskopischer** Biegelasten.
- Endurance-Konflikt: die 30 N aus den Flugversuchen waren durch EDF + schlechte
  Lötstellen überhöht; echter Reiseflugwiderstand bei L/D > 10 ist ~13–16 N.

## Dateien

```
bwb_propulsion_optimizer/
  aero.py        Zelle/Mission: Schubbedarf über Geschwindigkeit
  propeller.py   BasePropeller + MeasuredPropeller (APC-CSV) + EstimatedPropeller
  motor.py       BLDC-Ersatzschaltbild (inkl. io_voltage)
  battery.py     12S-Pack-Spannung
  solver.py      Betriebspunkt, Vollgas, Vmax, Standschub, Endurance
  optimizer.py   Kombinationssuche + Ranking + Entscheidungsgrößen
  database.py    PyThrust-JSON/CSV-Loader + tolerante CSV + Beispieldaten
run_bwb.py       Beispiellauf
```

## Grenzen

ESC-Wirkungsgrad/Kabelverluste nicht modelliert (als Faktor ergänzbar);
Reynolds-Skalierung nur über die in der CSV vorhandenen RPM-Blöcke; Endurance ist
eine Wh-Bilanz ohne Entlade-Kennlinie. Die mitgelieferten Beispielmotoren sind
**synthetische Platzhalter** — für belastbare Zahlen die echte PyThrust-DB laden.
