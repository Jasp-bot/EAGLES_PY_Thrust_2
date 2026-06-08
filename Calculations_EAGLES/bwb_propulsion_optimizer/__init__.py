"""
bwb_propulsion_optimizer
========================
Auswahl effizienter Motor-/Propeller-Kombinationen fuer ein Blended-Wing-Body
UAV in Pusher-Konfiguration. Orientiert sich an PyThrust (Beiwert-Solver +
Datenbanksuche) und an den Auslegungserkenntnissen des STEVE-Verlaufs.
"""
from .aero import Airframe
from .battery import Battery
from .motor import Motor
from .propeller import (BasePropeller, MeasuredPropeller, EstimatedPropeller,
                        estimated_from_inches)
from .solver import (evaluate, OperatingPoint, vmax, static_max_thrust,
                     full_throttle_point, endurance, ideal_kv)
from .optimizer import optimize, format_table, Candidate, filter_components
from .filters import Filters
from .progress import ProgressBar
from .plots import make_plots
from .database import (load_motor_json, load_propeller_json,
                       load_motors_dir, load_props_dir, load_motors_csv,
                       example_motors, example_props)

__version__ = "0.5.0"
__all__ = [
    "Airframe", "Battery", "Motor",
    "BasePropeller", "MeasuredPropeller", "EstimatedPropeller",
    "estimated_from_inches",
    "evaluate", "OperatingPoint", "vmax", "static_max_thrust",
    "full_throttle_point", "endurance", "ideal_kv",
    "optimize", "format_table", "Candidate", "filter_components", "Filters",
    "ProgressBar", "make_plots",
    "load_motor_json", "load_propeller_json", "load_motors_dir",
    "load_props_dir", "load_motors_csv", "example_motors", "example_props",
]
