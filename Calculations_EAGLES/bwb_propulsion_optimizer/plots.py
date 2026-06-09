"""
Grafische Auswertung der Optimizer-Ergebnisse (matplotlib).

Erzeugt im Stil des STEVE-Reports:
  1) Schub vs. Widerstand ueber Geschwindigkeit (Top-N): Vollgas-Schub jeder
     Kombi gegen den Widerstand -> Startueberschuss, Vmax als Schnittpunkt.
  2) Strahltheorie: idealer Froude-Wirkungsgrad + Strahl-Austrittsgeschwindigkeit
     ueber dem Durchmesser (warum grosser Durchmesser).
  3) Auswahl-Streudiagramme (wie AX435-Abbildung): eta_prop vs. Vmax und
     eta_prop vs. Kv-Abweichung, eingefaerbt nach Durchmesser, Top-N markiert.
  4) Cruise-Wirkungsgrad-Landkarte ueber Durchmesser x Steigung (Schaetzer),
     mit P/D-Linien und den Top-N als Sterne.
  5) Top-N-Vergleich (Balken): el. Leistung, eta_ges, Lastpunkt, Vmax.

matplotlib ist optional (PyThrust-"plot"-Extra). Fehlt es:  pip install matplotlib
"""

from __future__ import annotations
import math
import os
import numpy as np

from .propeller import EstimatedPropeller, IN2M
from .solver import full_throttle_point


def _require_mpl():
    try:
        import matplotlib.pyplot as plt
        return plt
    except ImportError as e:
        raise ImportError("matplotlib wird fuer die Plots benoetigt: "
                          "pip install matplotlib") from e


def _diam_color_map(plt, diams_in):
    levels = sorted({round(d) for d in diams_in})
    cmap = plt.get_cmap("tab10")
    return {lv: cmap(i % 10) for i, lv in enumerate(levels)}, levels


def _short(c):
    return f"{c.motor.name}/{c.prop.name}"


# ---------------------------------------------------------------- Plot 1
def plot_thrust_vs_drag(plt, cands, airframe, battery, pusher_factor,
                        v_max_target, bungee_speed, ax):
    rho = airframe.rho
    vmax_plot = (v_max_target or 30) + 12
    vs = np.linspace(1.0, vmax_plot, 90)
    drag = np.array([airframe.drag(v) for v in vs])
    ax.plot(vs, drag, color="crimson", lw=2.4, label="Widerstand (L/D)", zorder=5)

    tmax = 0.0
    cmap = plt.get_cmap("viridis")
    for i, c in enumerate(cands):
        th = []
        for v in vs:
            p = full_throttle_point(c.prop, c.motor, battery, v, rho)
            th.append(p["thrust"] * pusher_factor if p else np.nan)
        th = np.array(th)
        tmax = max(tmax, np.nanmax(th))
        col = cmap(i / max(len(cands) - 1, 1))
        ax.plot(vs, th, "--", color=col, lw=1.6, label=_short(c))
        if c.vmax:
            ax.plot(c.vmax, airframe.drag(c.vmax), "o", color=col, ms=6, zorder=6)

    for x, lab, col in ((airframe.v_cruise, "Cruise", "0.4"),
                        (bungee_speed, "Bungee-Exit", "green"),
                        (v_max_target, "Vmax-Ziel", "purple")):
        if x:
            ax.axvline(x, ls=":", color=col, lw=1)
            ax.text(x, 0.97, f" {lab}", rotation=90, va="top", ha="left",
                    color=col, fontsize=8, transform=ax.get_xaxis_transform())
    ax.set_xlim(0, vmax_plot)
    ax.set_ylim(0, (tmax * 1.15) if tmax else 80)   # auf Schubbereich begrenzen
    ax.set_xlabel("Fluggeschwindigkeit V [m/s]")
    ax.set_ylabel("Schub / Widerstand [N]")
    ax.set_title("Schub (Vollgas) vs. Widerstand — Vmax = Schnittpunkt")
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(alpha=0.3)


# ---------------------------------------------------------------- Plot 2
def plot_jet_theory(plt, airframe, thrusts, dmin_in, dmax_in, ax):
    rho, V0 = airframe.rho, airframe.v_cruise
    D_in = np.linspace(max(dmin_in - 2, 8), dmax_in + 3, 100)
    ax2 = ax.twinx()
    for T, style in zip(thrusts, ("-", "--")):
        A = math.pi * (D_in * IN2M / 2.0) ** 2
        w = 0.5 * (-V0 + np.sqrt(V0 ** 2 + 2 * T / (rho * A)))
        eta = V0 / (V0 + w)
        vjet = V0 + 2 * w
        ax.plot(D_in, eta, style, color="tab:blue",
                label=f"η_Froude @ {T:.0f} N")
        ax2.plot(D_in, vjet, style, color="tab:red",
                 label=f"V_Strahl @ {T:.0f} N")
    ax.axvline(dmax_in, color="0.3", lw=1)
    ax.text(dmax_in, 0.02, f' {dmax_in:.0f}" Limit', rotation=90, va="bottom",
            color="0.3", fontsize=8, transform=ax.get_xaxis_transform())
    ax2.axhline(V0, color="green", ls="-.", lw=1)
    ax2.text(D_in[0], V0, f" V0={V0:.0f} m/s", color="green", va="bottom",
             fontsize=8)
    ax.set_xlabel("Propellerdurchmesser [inch]")
    ax.set_ylabel("idealer Froude-Wirkungsgrad [-]", color="tab:blue")
    ax2.set_ylabel("Strahl-Austrittsgeschw. [m/s]", color="tab:red")
    ax.set_title("Strahltheorie: grosser Durchmesser → höherer Wirkungsgrad")
    ax.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7, loc="lower right")


# ---------------------------------------------------------------- Plot 3+4
def _scatter_selection(plt, ax, all_c, top, cmap, levels, x_of, xlabel,
                       vline=None, band=None, label_top=True):
    # Hintergrund vektorisiert: ein scatter-Aufruf je Durchmessergruppe
    # (NICHT pro Punkt -> sonst extrem langsam bei grossen Katalogen).
    import numpy as _np
    by_d = {}
    for c in all_c:
        d = round(c.prop.diameter_m / IN2M)
        by_d.setdefault(d, [[], []])
        by_d[d][0].append(x_of(c))
        by_d[d][1].append(c.cruise.eta_prop)
    for d, (xs, ys) in by_d.items():
        ax.scatter(_np.asarray(xs, float), _np.asarray(ys, float),
                   s=26, color=cmap[d], alpha=0.7, edgecolors="none", zorder=3)
    # Top-N als Sterne (ein Aufruf) + Beschriftung
    if top:
        tx = _np.array([x_of(c) for c in top], float)
        ty = _np.array([c.cruise.eta_prop for c in top], float)
        ax.scatter(tx, ty, s=130, marker="*", color="gold",
                   edgecolors="black", lw=0.7, zorder=5)
        if label_top:
            for k, (c, xx, yy) in enumerate(zip(top, tx, ty)):
                ax.annotate(_short(c), (xx, yy), fontsize=6.5,
                            xytext=(6, 6 + 12 * k), textcoords="offset points",
                            arrowprops=dict(arrowstyle="-", lw=0.4, color="0.5"))
    if vline is not None:
        ax.axvline(vline, ls="--", color="purple", lw=1)
    if band is not None:
        ax.axvspan(-band, band, color="green", alpha=0.10)
        ax.axvline(0, color="0.4", ls=":", lw=1)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Cruise-Wirkungsgrad η_prop")
    ax.grid(alpha=0.3)


def plot_selection(plt, all_c, top, v_max_target, kv_band=35.0,
                   max_bg_points=8000):
    # Defensiv: Hintergrund auf max_bg_points ausduennen (Top-N bleibt immer).
    if len(all_c) > max_bg_points:
        step = len(all_c) / max_bg_points
        idx = {int(i * step) for i in range(max_bg_points)}
        bg = [c for k, c in enumerate(all_c) if k in idx]
    else:
        bg = all_c
    diams = [c.prop.diameter_m / IN2M for c in bg]
    cmap, levels = _diam_color_map(plt, diams)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.2))
    # (1) eta_prop vs Kv-Abweichung
    _scatter_selection(plt, a1, bg, top, cmap, levels,
                       x_of=lambda c: c.motor.kv - c.kv_ideal,
                       xlabel="Kv-Abweichung (Motor-Kv − ideal); negativ = "
                              "Prop will weniger Kv",
                       band=kv_band)
    a1.set_title("Effizienz vs. Kv-Passung")
    # (2) eta_prop vs Vmax
    _scatter_selection(plt, a2, bg, top, cmap, levels,
                       x_of=lambda c: (c.vmax or np.nan),
                       xlabel="Vmax [m/s]", vline=v_max_target, label_top=False)
    a2.set_title("Effizienz vs. Vmax")
    handles = [plt.Line2D([], [], marker="o", ls="", color=cmap[d],
                          label=f'{d}"') for d in levels]
    handles.append(plt.Line2D([], [], marker="*", ls="", color="gold",
                              markeredgecolor="black", label="Top-N"))
    a2.legend(handles=handles, title="Durchmesser", fontsize=7, loc="best")
    fig.suptitle("Propeller-/Motor-Auswahl: Effizienz vs. Kv-Passung und Vmax",
                 fontweight="bold")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------- Plot 5
def plot_eta_landscape(plt, airframe, top, pusher_factor, dmin_in, dmax_in,
                       eta_max=0.78):
    rho, V0 = airframe.rho, airframe.v_cruise
    t_prop = airframe.thrust_required(V0) / pusher_factor
    D = np.linspace(dmin_in, dmax_in, 28)
    Pt = np.linspace(5, 22, 28)
    Z = np.full((len(Pt), len(D)), np.nan)
    for j, d in enumerate(D):
        for i, p in enumerate(Pt):
            pr = EstimatedPropeller("g", d * IN2M, p * IN2M, 2, eta_max=eta_max)
            sol = pr.solve_rps_for_thrust(t_prop, V0, rho)
            if sol:
                Z[i, j] = sol["eta_prop"]
    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    cf = ax.contourf(D, Pt, Z, levels=14, cmap="viridis")
    fig.colorbar(cf, ax=ax, label="Cruise-Wirkungsgrad η_prop (Schätzer)")
    for pd, ls in ((0.6, ":"), (0.7, "--")):
        ax.plot(D, pd * D, ls, color="black", lw=1, alpha=0.8)
        ax.text(D[-1], pd * D[-1], f" P/D={pd}", color="black", fontsize=7,
                va="center")
    for c in top:
        d_in = c.prop.diameter_m / IN2M
        p_in = c.prop.pitch_m / IN2M
        ax.plot(d_in, p_in, "*", color="red", ms=13, mec="white", mew=0.6)
        ax.annotate(_short(c), (d_in, p_in), color="red", fontsize=6.5,
                    xytext=(5, 4), textcoords="offset points")
    ax.set_xlabel("Durchmesser [inch]")
    ax.set_ylabel("Steigung (Pitch) [inch]")
    ax.set_title("Cruise-Wirkungsgrad-Landkarte (Orientierung, Schätzer)")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------- Plot 6
def plot_top_summary(plt, top):
    labels = [_short(c) for c in top]
    x = np.arange(len(top))
    fig, axs = plt.subplots(2, 2, figsize=(11, 7))
    data = [
        ("el. Cruise-Leistung [W]", [c.cruise.p_elec for c in top], "tab:red"),
        ("η_ges (System) [-]", [c.cruise.eta_total for c in top], "tab:green"),
        ("Motor-Lastpunkt [%]", [c.load_point * 100 for c in top], "tab:blue"),
        ("Vmax [m/s]", [(c.vmax or 0) for c in top], "tab:orange"),
    ]
    for ax, (title, vals, col) in zip(axs.ravel(), data):
        ax.bar(x, vals, color=col, alpha=0.85)
        ax.set_title(title, fontsize=10)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=7)
        ax.grid(alpha=0.3, axis="y")
        for xi, v in zip(x, vals):
            ax.text(xi, v, f"{v:.0f}" if v >= 10 else f"{v:.2f}",
                    ha="center", va="bottom", fontsize=7)
        if "Lastpunkt" in title:
            ax.axhspan(40, 70, color="green", alpha=0.12)
    fig.suptitle("Top-Kombinationen im Vergleich", fontweight="bold")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------- Plot 6
def plot_jet_velocity_vs_speed(plt, cands, airframe, pusher_factor,
                               v_max_target, ax):
    """Luftgeschwindigkeit am Rotoraustritt (Slipstream, V_jet = V + 2w) ueber
    der Fluggeschwindigkeit, im stationaeren Horizontalflug (T = Widerstand)."""
    import numpy as _np
    rho = airframe.rho
    vmax_plot = (v_max_target or 30) + 12
    vs = _np.linspace(6.0, vmax_plot, 120)
    cmap = plt.get_cmap("viridis")
    # nach Propeller entdoppeln (gleiche Scheibe -> gleiche Kurve)
    seen, uniq = set(), []
    for c in cands:
        key = round(c.prop.diameter_m, 4)
        if key not in seen:
            seen.add(key)
            uniq.append(c)
    jet_cruise = []
    for i, c in enumerate(uniq):
        A = c.prop.disk_area
        T = _np.array([airframe.drag(v) / pusher_factor for v in vs])
        w = 0.5 * (-vs + _np.sqrt(vs ** 2 + 2 * T / (rho * A)))
        vjet = vs + 2 * w
        col = cmap(i / max(len(uniq) - 1, 1))
        d_in = c.prop.diameter_m / IN2M
        ax.plot(vs, vjet, color=col, lw=1.9, label=f'{c.prop.name} (D={d_in:.0f}")')
        # Wert am Reisepunkt fuer die y-Skalierung
        Tc = airframe.drag(airframe.v_cruise) / pusher_factor
        wc = 0.5 * (-airframe.v_cruise + (airframe.v_cruise ** 2
                    + 2 * Tc / (rho * A)) ** 0.5)
        jet_cruise.append(airframe.v_cruise + 2 * wc)
    # Referenz: V_jet = V (kein Schub) und Winkelhalbierende
    ax.plot(vs, vs, "k:", lw=1.2, label="V_jet = V (kein Schub)")
    for x, lab, col in ((airframe.v_cruise, "Cruise", "0.4"),
                        (v_max_target, "Vmax-Ziel", "purple")):
        if x:
            ax.axvline(x, ls=":", color=col, lw=1)
            ax.text(x, 0.97, f" {lab}", rotation=90, va="top", ha="left",
                    color=col, fontsize=8, transform=ax.get_xaxis_transform())
    top = max(jet_cruise) * 1.7 if jet_cruise else 60
    ax.set_xlim(0, vmax_plot)
    ax.set_ylim(0, top)
    ax.set_xlabel("Fluggeschwindigkeit V [m/s]")
    ax.set_ylabel("Austrittsgeschwindigkeit am Rotor V_jet [m/s]")
    ax.set_title("Slipstream-Austrittsgeschwindigkeit über der Fluggeschwindigkeit\n"
                 "(stationärer Horizontalflug, T = Widerstand)")
    ax.legend(fontsize=7, loc="upper left")
    ax.grid(alpha=0.3)


# ---------------------------------------------------------------- Orchestrator
def make_plots(all_cands, airframe, battery, *, pusher_factor=0.92,
               v_max_target=30.0, bungee_speed=14.0,
               dmin_in=15.0, dmax_in=20.0, eta_max=0.78,
               top=5, outdir="plots", show=False) -> list[str]:
    """Erzeugt alle Plots fuer die besten `top` Kombinationen. Gibt die
    gespeicherten Dateipfade zurueck."""
    plt = _require_mpl()
    if not all_cands:
        print("[Plots] Keine Kombinationen vorhanden -> keine Plots.")
        return []
    import time as _time
    top_c = all_cands[:top]
    os.makedirs(outdir, exist_ok=True)
    paths = []
    print(f"[Plots] erzeuge 6 Abbildungen "
          f"({len(all_cands)} Kombis im Hintergrund, Top {len(top_c)}) ...")

    def _do(idx, name, fname, builder):
        t = _time.time()
        print(f"  [{idx}/6] {name} ...", end="", flush=True)
        try:
            fig = builder()
            p = os.path.join(outdir, fname)
            fig.savefig(p, dpi=130)
            if not show:
                plt.close(fig)
            paths.append(p)
            print(f" ok ({_time.time()-t:.1f}s)")
        except Exception as e:                      # ein Plot scheitert -> Rest laeuft
            print(f" FEHLER: {e}")

    def _b1():
        fig, ax = plt.subplots(figsize=(10, 5.6))
        plot_thrust_vs_drag(plt, top_c, airframe, battery, pusher_factor,
                            v_max_target, bungee_speed, ax)
        fig.tight_layout(); return fig

    def _b2():
        fig, ax = plt.subplots(figsize=(8, 5.2))
        tc = airframe.thrust_required(airframe.v_cruise)
        plot_jet_theory(plt, airframe, [tc, tc * 1.7], dmin_in, dmax_in, ax)
        fig.tight_layout(); return fig

    def _b6():
        fig, ax = plt.subplots(figsize=(9, 5.6))
        plot_jet_velocity_vs_speed(plt, top_c, airframe, pusher_factor,
                                   v_max_target, ax)
        fig.tight_layout(); return fig

    _do(1, "Schub vs. Widerstand", "1_schub_vs_widerstand.png", _b1)
    _do(2, "Strahltheorie", "2_strahltheorie.png", _b2)
    _do(3, "Auswahl-Streudiagramme", "3_auswahl_streudiagramme.png",
        lambda: plot_selection(plt, all_cands, top_c, v_max_target))
    _do(4, "Cruise-η-Landkarte", "4_eta_landkarte.png",
        lambda: plot_eta_landscape(plt, airframe, top_c, pusher_factor,
                                   dmin_in, dmax_in, eta_max))
    _do(5, "Top-Vergleich", "5_top_vergleich.png",
        lambda: plot_top_summary(plt, top_c))
    _do(6, "Austrittsgeschwindigkeit über V", "6_austrittsgeschwindigkeit.png", _b6)

    if show:
        plt.show()
    return paths
