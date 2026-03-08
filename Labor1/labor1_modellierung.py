"""
Labor 1 – Physikalische Modellierung eines Gleichstrommotors
============================================================
Herleitung des mathematischen Modells, Zustandsraumdarstellung,
Übertragungsfunktion und Sprungantwort-Simulation.

Ergebnis wird in data/modell_parameter.json gespeichert und
von Labor 2 und Labor 3 verwendet.
"""

import json
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import lti, step


# ---------------------------------------------------------------------------
# 1. Modellparameter des DC-Motors (nominale Werte)
# ---------------------------------------------------------------------------
MOTOR_PARAMS = {
    "R": 1.0,      # Ankerwiderstand [Ohm]
    "L": 0.01,     # Ankerinduktivität [H]
    "K_e": 0.1,    # Gegen-EMK-Konstante [V·s/rad]
    "K_t": 0.1,    # Drehmomentkonstante [N·m/A]
    "J": 0.005,    # Trägheitsmoment [kg·m²]
    "b": 0.001,    # Reibungskoeffizient [N·m·s/rad]
}


def berechne_zustandsraum(p):
    """Berechnet die Zustandsraummatrizen A, B, C, D des DC-Motors.

    Zustandsvektor: x = [i, omega]^T
    Eingang: u (Spannung)
    Ausgang: y = omega (Winkelgeschwindigkeit)
    """
    R, L, K_e, K_t, J, b = p["R"], p["L"], p["K_e"], p["K_t"], p["J"], p["b"]

    A = np.array([
        [-R / L,   -K_e / L],
        [K_t / J,  -b / J  ],
    ])
    B = np.array([[1.0 / L], [0.0]])
    C = np.array([[0.0, 1.0]])
    D = np.array([[0.0]])

    return A, B, C, D


def berechne_uebertragungsfunktion(p):
    """Berechnet Zähler- und Nennerkoeffizienten der Übertragungsfunktion
    G(s) = omega(s)/U(s) für den DC-Motor.

    G(s) = K_t / (L*J*s^2 + (R*J + b*L)*s + (R*b + K_e*K_t))
    """
    R, L, K_e, K_t, J, b = p["R"], p["L"], p["K_e"], p["K_t"], p["J"], p["b"]

    zaehler = [K_t]
    nenner = [
        L * J,
        R * J + b * L,
        R * b + K_e * K_t,
    ]
    return zaehler, nenner


def simuliere_sprungantwort(zaehler, nenner, t_end=2.0, u_amplitude=12.0):
    """Simuliert die Sprungantwort des Systems für einen Spannungssprung."""
    system = lti(
        [c * u_amplitude for c in zaehler],
        nenner,
    )
    t, y = step(system, T=np.linspace(0, t_end, 1000))
    return t, y


def berechne_kennwerte(t, y):
    """Berechnet charakteristische Kennwerte der Sprungantwort."""
    omega_end = y[-1]
    # Anstiegszeit (10% → 90% des stationären Wertes)
    idx_10 = np.searchsorted(y, 0.10 * omega_end)
    idx_90 = np.searchsorted(y, 0.90 * omega_end)
    t_anstieg = t[idx_90] - t[idx_10] if idx_90 < len(t) else float("nan")

    # Einregelzeit (±2% Band)
    toleranz = 0.02 * omega_end
    idx_einregel = len(t) - 1
    for i in range(len(t) - 1, -1, -1):
        if abs(y[i] - omega_end) > toleranz:
            idx_einregel = i + 1
            break
    t_einregel = t[idx_einregel] if idx_einregel < len(t) else float("nan")

    return {
        "omega_stationaer": float(omega_end),
        "t_anstieg_s": float(t_anstieg),
        "t_einregel_s": float(t_einregel),
    }


def main():
    print("=" * 60)
    print("Labor 1 – Physikalische Modellierung des DC-Motors")
    print("=" * 60)

    p = MOTOR_PARAMS

    # --- Zustandsraummodell --------------------------------------------------
    A, B, C, D = berechne_zustandsraum(p)
    print("\nZustandsraummodell:")
    print(f"  A =\n{A}")
    print(f"  B =\n{B}")
    print(f"  C = {C}")
    print(f"  D = {D}")

    # --- Übertragungsfunktion ------------------------------------------------
    zaehler, nenner = berechne_uebertragungsfunktion(p)
    print(f"\nÜbertragungsfunktion G(s) = omega(s)/U(s):")
    print(f"  Zähler:  {zaehler}")
    print(f"  Nenner:  {nenner}")

    # --- Sprungantwort -------------------------------------------------------
    u_amplitude = 12.0  # Volt
    t, y = simuliere_sprungantwort(zaehler, nenner, t_end=2.0, u_amplitude=u_amplitude)
    kennwerte = berechne_kennwerte(t, y)

    print(f"\nSprungantwort bei U = {u_amplitude} V:")
    print(f"  Stationäre Winkelgeschwindigkeit: {kennwerte['omega_stationaer']:.4f} rad/s")
    print(f"  Anstiegszeit (10%→90%):           {kennwerte['t_anstieg_s']:.4f} s")
    print(f"  Einregelzeit (±2%):               {kennwerte['t_einregel_s']:.4f} s")

    # --- Plot ----------------------------------------------------------------
    os.makedirs("data", exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, y, "b-", linewidth=2, label="Winkelgeschwindigkeit ω(t)")
    ax.axhline(kennwerte["omega_stationaer"], color="r", linestyle="--",
               label=f"Stationärwert = {kennwerte['omega_stationaer']:.2f} rad/s")
    ax.set_xlabel("Zeit t [s]")
    ax.set_ylabel("ω [rad/s]")
    ax.set_title(f"Labor 1 – Sprungantwort des DC-Motors (U = {u_amplitude} V)")
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig("data/sprungantwort_modell.png", dpi=150)
    plt.show()
    print("\nPlot gespeichert: data/sprungantwort_modell.png")

    # --- Daten speichern (für Labor 2 und Labor 3) --------------------------
    ausgabe = {
        "beschreibung": "Nominale DC-Motor-Parameter aus Labor 1",
        "parameter": p,
        "uebertragungsfunktion": {
            "zaehler": zaehler,
            "nenner": nenner,
        },
        "zustandsraum": {
            "A": A.tolist(),
            "B": B.tolist(),
            "C": C.tolist(),
            "D": D.tolist(),
        },
        "sprungantwort_kennwerte": kennwerte,
        "sprung_amplitude_V": u_amplitude,
    }
    with open("data/modell_parameter.json", "w", encoding="utf-8") as f:
        json.dump(ausgabe, f, indent=4, ensure_ascii=False)

    print("Modellparameter gespeichert: data/modell_parameter.json")
    print("\nLabor 1 abgeschlossen. Weiter mit Labor 2.")


if __name__ == "__main__":
    main()
