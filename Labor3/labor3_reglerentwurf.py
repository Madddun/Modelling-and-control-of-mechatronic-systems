"""
Labor 3 – Reglerentwurf für den DC-Motor
=========================================
Entwurf eines PID-Reglers basierend auf dem in Labor 1 hergeleiteten
und in Labor 2 identifizierten Modell des DC-Motors.

Voraussetzungen:
    - Labor 1 muss ausgeführt worden sein:
      ../Labor1/data/modell_parameter.json
    - Labor 2 muss ausgeführt worden sein:
      ../Labor2/data/identifizierte_parameter.json

Ergebnis wird in data/regler_ergebnisse.json gespeichert.
"""

import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import lti, step

# Pfade zu den Daten aus Labor 1 und Labor 2
LABOR1_DATEN = os.path.join(os.path.dirname(__file__), "..", "Labor1", "data", "modell_parameter.json")
LABOR2_DATEN = os.path.join(os.path.dirname(__file__), "..", "Labor2", "data", "identifizierte_parameter.json")


def lade_daten():
    """Lädt die Modellparameter aus Labor 1 und Labor 2."""
    fehlend = []
    if not os.path.exists(LABOR1_DATEN):
        fehlend.append(f"Labor 1: {LABOR1_DATEN}")
    if not os.path.exists(LABOR2_DATEN):
        fehlend.append(f"Labor 2: {LABOR2_DATEN}")

    if fehlend:
        print("FEHLER: Folgende Datendateien fehlen:")
        for f in fehlend:
            print(f"  - {f}")
        print("\nBitte vorherige Labs ausführen:")
        print("  cd ../Labor1 && python labor1_modellierung.py")
        print("  cd ../Labor2 && python labor2_systemidentifikation.py")
        sys.exit(1)

    with open(LABOR1_DATEN, "r", encoding="utf-8") as f:
        labor1 = json.load(f)
    with open(LABOR2_DATEN, "r", encoding="utf-8") as f:
        labor2 = json.load(f)

    return labor1, labor2


def pid_regler_ziegler_nichols(zaehler, nenner):
    """Berechnet PID-Parameter nach vereinfachter Ziegler-Nichols-Methode.

    Für PT2-Strecken wird die Methode auf Basis der Sprungantwort angewendet:
    - Wendetangente → Tu (Verzugszeit), Tg (Ausgleichszeit)
    - Daraus: Kp, Ti, Td nach Ziegler-Nichols-Tabelle
    """
    t = np.linspace(0, 5.0, 5000)
    system = lti(zaehler, nenner)
    _, y = step(system, T=t)
    y_end = y[-1]

    if y_end == 0:
        return {"Kp": 1.0, "Ti": 1.0, "Td": 0.1}

    y_norm = y / y_end

    # Wendetangente: Punkt maximaler Steigung
    dy = np.gradient(y_norm, t)
    idx_wende = np.argmax(dy)
    t_wende = t[idx_wende]
    y_wende = y_norm[idx_wende]
    steigung = dy[idx_wende]

    # Schnittpunkte der Wendetangente mit y=0 und y=1
    t_u = t_wende - y_wende / steigung  # Schnittpunkt y=0
    t_g = t_wende + (1.0 - y_wende) / steigung  # Schnittpunkt y=1

    # Verzugs- und Ausgleichszeit (mindestens kleine positive Werte)
    Tu = max(t_u, 0.001)
    Tg = max(t_g - t_u, 0.001)

    # Ziegler-Nichols PID-Parameter (Einheitsrückführung)
    K_strecke = y_end  # Statische Verstärkung der Strecke
    Kp = 1.2 * Tg / (K_strecke * Tu)
    Ti = 2.0 * Tu
    Td = 0.5 * Tu

    return {"Kp": Kp, "Ti": Ti, "Td": Td}


def simuliere_pid_regelkreis(zaehler_strecke, nenner_strecke, Kp, Ti, Td,
                              w=1.0, t_end=5.0, n_punkte=5000):
    """Simuliert den geschlossenen PID-Regelkreis numerisch (Euler-Methode).

    Regler: C(s) = Kp * (1 + 1/(Ti*s) + Td*s)
    Strecke: G(s)

    Implementierung als Zeitbereichs-Simulation (Vorwärts-Euler).
    """
    dt = t_end / n_punkte
    t = np.linspace(0, t_end, n_punkte)

    # Strecke als LTI-System für interne Simulation
    strecke = lti(zaehler_strecke, nenner_strecke)
    # Zustandsraumdarstellung
    sys_ss = strecke.to_ss()
    A = sys_ss.A
    B = sys_ss.B
    C = sys_ss.C
    D = sys_ss.D

    n_states = A.shape[0]
    x = np.zeros(n_states)

    y_arr = np.zeros(n_punkte)
    u_arr = np.zeros(n_punkte)
    e_arr = np.zeros(n_punkte)

    integral = 0.0
    e_prev = 0.0

    for k in range(n_punkte):
        # Output from current state (D=0 for DC motor, D·u term vanishes)
        y = float((C @ x).flatten()[0])
        e = w - y
        integral += e * dt
        derivativ = (e - e_prev) / dt if k > 0 else 0.0
        e_prev = e

        u = Kp * (e + integral / Ti + Td * derivativ)
        u = np.clip(u, -100.0, 100.0)  # Stellgrößenbeschränkung

        y_arr[k] = y
        u_arr[k] = u
        e_arr[k] = e

        # Euler-Integration des Zustandsvektors
        x = x + dt * (A @ x + B.flatten() * u)

    return t, y_arr, u_arr, e_arr


def berechne_regelguete(t, y, w):
    """Berechnet Kennzahlen der Regelgüte."""
    y_end = y[-1]
    ueberschwingen = max(0.0, (np.max(y) - w) / w * 100.0) if w != 0 else 0.0

    # Einregelzeit (±2% Band um w)
    toleranz = 0.02 * abs(w)
    idx_einregel = len(t) - 1
    for i in range(len(t) - 1, -1, -1):
        if abs(y[i] - w) > toleranz:
            idx_einregel = i + 1
            break
    t_einregel = t[idx_einregel] if idx_einregel < len(t) else float(t[-1])

    # Stationäre Abweichung
    e_stationaer = abs(w - y_end)

    # Anstiegszeit (0% → 90% von w)
    idx_90 = np.searchsorted(y, 0.90 * w)
    t_anstieg = t[min(idx_90, len(t) - 1)]

    return {
        "ueberschwingen_prozent": float(ueberschwingen),
        "einregelzeit_s": float(t_einregel),
        "stationaere_abweichung": float(e_stationaer),
        "anstiegszeit_s": float(t_anstieg),
        "stationaerer_ausgangswert": float(y_end),
    }


def bode_daten(zaehler, nenner, omega_min=0.1, omega_max=1e4, n=500):
    """Berechnet Bode-Diagramm-Daten (Betrag und Phase)."""
    omega = np.logspace(np.log10(omega_min), np.log10(omega_max), n)
    s = 1j * omega
    G = np.polyval(zaehler, s) / np.polyval(nenner, s)
    betrag_db = 20.0 * np.log10(np.abs(G))
    phase_deg = np.angle(G, deg=True)
    return omega, betrag_db, phase_deg


def main():
    print("=" * 60)
    print("Labor 3 – Reglerentwurf für den DC-Motor")
    print("=" * 60)

    # --- Daten aus Labor 1 und 2 laden --------------------------------------
    labor1, labor2 = lade_daten()
    print(f"\nDaten aus Labor 1 geladen: {LABOR1_DATEN}")
    print(f"Daten aus Labor 2 geladen: {LABOR2_DATEN}")

    # Identifizierte Parameter (Labor 2) für Reglerentwurf verwenden
    zaehler_id = labor2["uebertragungsfunktion"]["zaehler"]
    nenner_id = labor2["uebertragungsfunktion"]["nenner"]
    zaehler_nom = labor1["uebertragungsfunktion"]["zaehler"]
    nenner_nom = labor1["uebertragungsfunktion"]["nenner"]

    p_id = labor2["parameter"]
    print("\nIdentifizierte Parameter (Labor 2) für Reglerentwurf:")
    for k, v in p_id.items():
        print(f"  {k} = {v:.6f}")

    # --- PID-Reglerentwurf (Ziegler-Nichols) --------------------------------
    pid = pid_regler_ziegler_nichols(zaehler_id, nenner_id)
    Kp, Ti, Td = pid["Kp"], pid["Ti"], pid["Td"]
    print(f"\nPID-Parameter (Ziegler-Nichols):")
    print(f"  Kp = {Kp:.4f}")
    print(f"  Ti = {Ti:.4f} s")
    print(f"  Td = {Td:.4f} s")

    # --- Simulation des Regelkreises ----------------------------------------
    w_soll = 100.0  # Sollwert: 100 rad/s
    t, y, u, e = simuliere_pid_regelkreis(
        zaehler_id, nenner_id, Kp, Ti, Td,
        w=w_soll, t_end=3.0, n_punkte=6000,
    )
    guete = berechne_regelguete(t, y, w_soll)

    print(f"\nRegelgüte (Sollwert w = {w_soll} rad/s):")
    print(f"  Überschwingen:          {guete['ueberschwingen_prozent']:.2f} %")
    print(f"  Einregelzeit (±2%):     {guete['einregelzeit_s']:.4f} s")
    print(f"  Anstiegszeit (0→90%):   {guete['anstiegszeit_s']:.4f} s")
    print(f"  Stationäre Abweichung:  {guete['stationaere_abweichung']:.4f} rad/s")

    # --- Bode-Diagramm der Strecke ------------------------------------------
    omega, betrag_nom, phase_nom = bode_daten(zaehler_nom, nenner_nom)
    _, betrag_id, phase_id = bode_daten(zaehler_id, nenner_id)

    # --- Plots --------------------------------------------------------------
    os.makedirs("data", exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Labor 3 – Reglerentwurf DC-Motor", fontsize=14)

    # Plot 1: Geschlossener Regelkreis – Sprungantwort
    ax1 = axes[0, 0]
    ax1.plot(t, y, "b-", linewidth=2, label=f"Ausgangsgröße ω(t)")
    ax1.axhline(w_soll, color="r", linestyle="--", label=f"Sollwert w = {w_soll} rad/s")
    ax1.axhline(w_soll * 1.02, color="gray", linestyle=":", alpha=0.5, label="±2% Band")
    ax1.axhline(w_soll * 0.98, color="gray", linestyle=":", alpha=0.5)
    ax1.set_xlabel("Zeit t [s]")
    ax1.set_ylabel("ω [rad/s]")
    ax1.set_title("Sprungantwort (geschlossener Regelkreis)")
    ax1.legend(fontsize=8)
    ax1.grid(True)

    # Plot 2: Stellgröße
    ax2 = axes[0, 1]
    ax2.plot(t, u, "g-", linewidth=1.5, label="Stellgröße u(t) [V]")
    ax2.set_xlabel("Zeit t [s]")
    ax2.set_ylabel("u [V]")
    ax2.set_title("Stellgröße des PID-Reglers")
    ax2.legend(fontsize=8)
    ax2.grid(True)

    # Plot 3: Bode-Diagramm Betrag
    ax3 = axes[1, 0]
    ax3.semilogx(omega, betrag_nom, "b-", linewidth=2, label="Nominales Modell (Labor 1)")
    ax3.semilogx(omega, betrag_id, "r--", linewidth=2, label="Identifiziertes Modell (Labor 2)")
    ax3.set_xlabel("Frequenz ω [rad/s]")
    ax3.set_ylabel("|G(jω)| [dB]")
    ax3.set_title("Bode-Diagramm – Betrag der Strecke")
    ax3.legend(fontsize=8)
    ax3.grid(True, which="both")

    # Plot 4: Bode-Diagramm Phase
    ax4 = axes[1, 1]
    ax4.semilogx(omega, phase_nom, "b-", linewidth=2, label="Nominales Modell (Labor 1)")
    ax4.semilogx(omega, phase_id, "r--", linewidth=2, label="Identifiziertes Modell (Labor 2)")
    ax4.axhline(-180, color="gray", linestyle=":", alpha=0.7)
    ax4.set_xlabel("Frequenz ω [rad/s]")
    ax4.set_ylabel("∠G(jω) [°]")
    ax4.set_title("Bode-Diagramm – Phase der Strecke")
    ax4.legend(fontsize=8)
    ax4.grid(True, which="both")

    plt.tight_layout()
    plt.savefig("data/regler_ergebnisse.png", dpi=150)
    plt.show()
    print("\nPlot gespeichert: data/regler_ergebnisse.png")

    # --- Ergebnisse speichern -----------------------------------------------
    ausgabe = {
        "beschreibung": "PID-Reglerentwurf basierend auf Labor 1 und Labor 2",
        "regler_parameter": {
            "typ": "PID",
            "methode": "Ziegler-Nichols (Wendetangente)",
            "Kp": Kp,
            "Ti": Ti,
            "Td": Td,
        },
        "regelguete": guete,
        "sollwert_rad_s": w_soll,
        "quellen": {
            "labor1": LABOR1_DATEN,
            "labor2": LABOR2_DATEN,
        },
    }

    with open("data/regler_ergebnisse.json", "w", encoding="utf-8") as f:
        json.dump(ausgabe, f, indent=4, ensure_ascii=False)

    print("Reglerergebnisse gespeichert: data/regler_ergebnisse.json")
    print("\nLabor 3 abgeschlossen.")


if __name__ == "__main__":
    main()
