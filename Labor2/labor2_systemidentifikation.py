"""
Labor 2 – Systemidentifikation des DC-Motors
=============================================
Schätzung der Systemparameter aus Messdaten (Sprungantwort).
Verwendet das nominale Modell aus Labor 1 als Referenz.

Ergebnis wird in data/identifizierte_parameter.json gespeichert und
von Labor 3 verwendet.

Voraussetzung:
    Labor 1 muss ausgeführt worden sein:
    ../Labor1/data/modell_parameter.json muss existieren.
"""

import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lti, step

# Pfad zu Labor 1 Daten
LABOR1_DATEN = os.path.join(os.path.dirname(__file__), "..", "Labor1", "data", "modell_parameter.json")


def lade_labor1_daten():
    """Lädt die Modellparameter aus Labor 1."""
    if not os.path.exists(LABOR1_DATEN):
        print(f"FEHLER: Labor 1 Daten nicht gefunden: {LABOR1_DATEN}")
        print("Bitte zuerst Labor 1 ausführen: cd ../Labor1 && python labor1_modellierung.py")
        sys.exit(1)
    with open(LABOR1_DATEN, "r", encoding="utf-8") as f:
        return json.load(f)


def simuliere_sprungantwort(params, t, u_amplitude):
    """Simuliert Sprungantwort für gegebene Parameter [R, L, K_e, K_t, J, b]."""
    R, L, K_e, K_t, J, b = params
    # Übertragungsfunktion G(s) = K_t / (L*J*s^2 + (R*J + b*L)*s + (R*b + K_e*K_t))
    zaehler = [K_t * u_amplitude]
    nenner = [L * J, R * J + b * L, R * b + K_e * K_t]
    try:
        system = lti(zaehler, nenner)
        _, y = step(system, T=t)
        return y
    except Exception:
        return np.zeros_like(t)


def erzeuge_messdaten(labor1_daten, rausch_niveau=0.05, seed=42):
    """Erzeugt synthetische Messdaten aus dem nominalen Modell mit Rauschen.

    In einem realen Labor würden hier die aufgezeichneten Messdaten
    (z.B. aus einer DAQ-Karte) geladen.
    """
    rng = np.random.default_rng(seed)
    p = labor1_daten["parameter"]
    u_amplitude = labor1_daten["sprung_amplitude_V"]

    params_nominal = [p["R"], p["L"], p["K_e"], p["K_t"], p["J"], p["b"]]
    t = np.linspace(0, 2.0, 500)
    y_nominal = simuliere_sprungantwort(params_nominal, t, u_amplitude)

    # Additives weißes Rauschen
    sigma = rausch_niveau * np.max(y_nominal)
    y_mess = y_nominal + rng.normal(0, sigma, size=len(t))
    return t, y_mess, u_amplitude


def residuen(params, t, y_mess, u_amplitude):
    """Residuenvektor für Least-Squares-Optimierung."""
    y_sim = simuliere_sprungantwort(params, t, u_amplitude)
    return y_sim - y_mess


def identifiziere_parameter(t, y_mess, u_amplitude, params_start):
    """Schätzt die Systemparameter durch nichtlineare Least-Squares-Optimierung."""
    # Alle Parameter müssen positiv sein
    grenzen_unten = [1e-4, 1e-6, 1e-4, 1e-4, 1e-6, 1e-6]
    grenzen_oben = [100.0, 10.0, 10.0, 10.0, 10.0, 10.0]

    ergebnis = least_squares(
        residuen,
        x0=params_start,
        args=(t, y_mess, u_amplitude),
        bounds=(grenzen_unten, grenzen_oben),
        method="trf",
        max_nfev=5000,
    )
    return ergebnis


def berechne_relative_fehler(params_nominal, params_id, namen):
    """Berechnet den relativen Fehler zwischen nominalen und identifizierten Parametern."""
    fehler = {}
    for name, p_nom, p_id in zip(namen, params_nominal, params_id):
        rel = abs(p_id - p_nom) / abs(p_nom) * 100.0
        fehler[name] = {"nominal": p_nom, "identifiziert": p_id, "fehler_prozent": rel}
    return fehler


def main():
    print("=" * 60)
    print("Labor 2 – Systemidentifikation des DC-Motors")
    print("=" * 60)

    # --- Labor 1 Daten laden ------------------------------------------------
    labor1_daten = lade_labor1_daten()
    p_nom = labor1_daten["parameter"]
    u_amplitude = labor1_daten["sprung_amplitude_V"]
    print(f"\nNominale Parameter aus Labor 1 geladen: {LABOR1_DATEN}")

    # --- Messdaten erzeugen (oder laden) ------------------------------------
    t, y_mess, u_amplitude = erzeuge_messdaten(labor1_daten, rausch_niveau=0.05)
    print(f"\nMessdaten erzeugt: {len(t)} Messpunkte, Rauschpegel ≈ 5%")

    # --- Startwerte für Optimierung (gestörte nominale Werte) ---------------
    rng = np.random.default_rng(99)
    namen = ["R", "L", "K_e", "K_t", "J", "b"]
    params_nominal = [p_nom[n] for n in namen]
    # Startschätzung mit ±20% Störung
    params_start = [v * (1.0 + rng.uniform(-0.20, 0.20)) for v in params_nominal]

    print("\nStartwerte für Optimierung (±20% gestört):")
    for n, v in zip(namen, params_start):
        print(f"  {n} = {v:.6f}")

    # --- Systemidentifikation -----------------------------------------------
    print("\nFühre Least-Squares-Optimierung durch ...")
    ergebnis = identifiziere_parameter(t, y_mess, u_amplitude, params_start)
    params_id = ergebnis.x

    print(f"Optimierung {'erfolgreich' if ergebnis.success else 'nicht konvergiert'}.")
    print(f"  Iterationen:    {ergebnis.nfev}")
    print(f"  Residuennorm:   {np.linalg.norm(ergebnis.fun):.6f}")

    # --- Ergebnisvergleich --------------------------------------------------
    fehler = berechne_relative_fehler(params_nominal, params_id, namen)
    print("\nVergleich nominale vs. identifizierte Parameter:")
    print(f"  {'Parameter':<6} {'Nominal':>12} {'Identifiziert':>15} {'Fehler':>10}")
    print(f"  {'-'*6} {'-'*12} {'-'*15} {'-'*10}")
    for n, info in fehler.items():
        print(f"  {n:<6} {info['nominal']:>12.6f} {info['identifiziert']:>15.6f} "
              f"{info['fehler_prozent']:>9.2f}%")

    # --- Plot ---------------------------------------------------------------
    y_nominal = simuliere_sprungantwort(params_nominal, t, u_amplitude)
    y_id = simuliere_sprungantwort(params_id, t, u_amplitude)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, y_mess, "gray", linewidth=0.8, alpha=0.6, label="Messdaten (mit Rauschen)")
    ax.plot(t, y_nominal, "b-", linewidth=2, label="Nominales Modell (Labor 1)")
    ax.plot(t, y_id, "r--", linewidth=2, label="Identifiziertes Modell (Labor 2)")
    ax.set_xlabel("Zeit t [s]")
    ax.set_ylabel("ω [rad/s]")
    ax.set_title("Labor 2 – Systemidentifikation: Vergleich nominales vs. identifiziertes Modell")
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig("data/systemidentifikation_vergleich.png", dpi=150)
    plt.show()
    print("\nPlot gespeichert: data/systemidentifikation_vergleich.png")

    # --- Daten speichern (für Labor 3) --------------------------------------
    os.makedirs("data", exist_ok=True)
    id_params_dict = {n: float(v) for n, v in zip(namen, params_id)}
    zaehler = [id_params_dict["K_t"]]
    nenner = [
        id_params_dict["L"] * id_params_dict["J"],
        id_params_dict["R"] * id_params_dict["J"] + id_params_dict["b"] * id_params_dict["L"],
        id_params_dict["R"] * id_params_dict["b"] + id_params_dict["K_e"] * id_params_dict["K_t"],
    ]
    ausgabe = {
        "beschreibung": "Identifizierte DC-Motor-Parameter aus Labor 2",
        "parameter": id_params_dict,
        "uebertragungsfunktion": {
            "zaehler": zaehler,
            "nenner": nenner,
        },
        "identifikation_info": {
            "methode": "Nichtlineare Least-Squares (scipy.optimize.least_squares, TRF)",
            "anzahl_messpunkte": len(t),
            "rausch_niveau_prozent": 5.0,
            "residuennorm": float(np.linalg.norm(ergebnis.fun)),
            "konvergiert": bool(ergebnis.success),
        },
        "parametervergleich": {
            n: {
                "nominal": info["nominal"],
                "identifiziert": info["identifiziert"],
                "fehler_prozent": info["fehler_prozent"],
            }
            for n, info in fehler.items()
        },
        "sprung_amplitude_V": u_amplitude,
        "labor1_referenz": LABOR1_DATEN,
    }

    with open("data/identifizierte_parameter.json", "w", encoding="utf-8") as f:
        json.dump(ausgabe, f, indent=4, ensure_ascii=False)

    print("Identifizierte Parameter gespeichert: data/identifizierte_parameter.json")
    print("\nLabor 2 abgeschlossen. Weiter mit Labor 3.")


if __name__ == "__main__":
    main()
