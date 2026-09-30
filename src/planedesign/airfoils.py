"""2D airfoil comparison at the plane's low Reynolds numbers, using NeuralFoil."""

from __future__ import annotations

import aerosandbox as asb
import aerosandbox.numpy as np


def reynolds(velocity: float, chord: float, altitude: float = 500.0) -> float:
    atmo = asb.Atmosphere(altitude=altitude)
    return float(atmo.density() * velocity * chord / atmo.dynamic_viscosity())


def polar(name: str, re: float, alphas=None, model_size: str = "large") -> dict:
    alphas = np.linspace(-4, 14, 73) if alphas is None else alphas
    af = asb.Airfoil(name)
    r = af.get_aero_from_neuralfoil(alpha=alphas, Re=re, model_size=model_size)
    return {
        "alpha": alphas,
        "CL": r["CL"],
        "CD": r["CD"],
        "CM": r["CM"],
        "confidence": r["analysis_confidence"],
    }


def summary(name: str, re: float) -> dict:
    p = polar(name, re)
    ld = p["CL"] / p["CD"]
    i_ld = int(np.argmax(ld))
    i_cl = int(np.argmax(p["CL"]))
    # endurance figure of merit CL^1.5/CD over the positive-lift range
    endurance = np.where(p["CL"] > 0, np.abs(p["CL"]) ** 1.5 / p["CD"], 0)
    return {
        "airfoil": name,
        "Re": re,
        "t/c": float(asb.Airfoil(name).max_thickness()),
        "CLmax": float(p["CL"][i_cl]),
        "alpha_CLmax": float(p["alpha"][i_cl]),
        "L/D_max": float(ld[i_ld]),
        "CL_at_L/D_max": float(p["CL"][i_ld]),
        "CL^1.5/CD_max": float(np.max(endurance)),
        "CM_at_CL0.5": float(np.interp(0.5, p["CL"][: i_cl + 1], p["CM"][: i_cl + 1])),
        "min_confidence": float(np.min(p["confidence"][: i_cl + 1])),
    }
