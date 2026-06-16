"""
Ergebnis-Protokollierung: pro Durchlauf ein versionierter Unterordner unter
Results/ mit den Plots und einer results.txt (Konfiguration + Ergebnisse).

- next_run_dir(): legt Results/run_<NNN>_<Datum>/ an und gibt (Pfad, Nummer).
- Reporter: sammelt die Konsolenzeilen fuer die Textdatei UND gibt sie aus.
  Fortschrittsbalken und Plot-Status laufen NICHT ueber den Reporter und landen
  daher nicht in der Datei (sie werden separat direkt geprintet).
"""

from __future__ import annotations
from pathlib import Path
from datetime import datetime


def next_run_dir(base_dir, prefix: str = "run") -> tuple[Path, int]:
    base = Path(base_dir)
    base.mkdir(parents=True, exist_ok=True)
    nums = []
    for d in base.iterdir():
        if d.is_dir() and d.name.startswith(prefix + "_"):
            tail = d.name[len(prefix) + 1:].split("_")[0]
            if tail.isdigit():
                nums.append(int(tail))
    n = (max(nums) + 1) if nums else 1
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M")
    run = base / f"{prefix}_{n:03d}_{stamp}"
    run.mkdir(parents=True, exist_ok=True)
    return run, n


class Reporter:
    """Tee: gibt Zeilen auf der Konsole aus und merkt sie fuer die Textdatei."""

    def __init__(self):
        self.lines: list[str] = []

    def print(self, *args):
        s = " ".join(str(a) for a in args)
        print(s)
        self.lines.append(s)

    def blank(self):
        print()
        self.lines.append("")

    def save(self, path, title: str | None = None):
        path = Path(path)
        with open(path, "w", encoding="utf-8") as fh:
            if title:
                fh.write(title + "\n" + "=" * len(title) + "\n\n")
            fh.write("\n".join(self.lines).rstrip() + "\n")
        return path
