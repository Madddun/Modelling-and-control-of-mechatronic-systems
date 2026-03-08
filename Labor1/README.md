# Labor 1 – Physikalische Modellierung eines Gleichstrommotors

## Ziel
In diesem Labor wird das mathematische Modell eines Gleichstrommotors (DC-Motor) hergeleitet.
Das Modell umfasst die elektrische und mechanische Dynamik des Systems.

## Systembeschreibung

### Elektrischer Teil
```
L * di/dt + R * i = u - K_e * omega
```

### Mechanischer Teil
```
J * domega/dt + b * omega = K_t * i
```

**Variablen:**
| Symbol | Bedeutung | Einheit |
|--------|-----------|---------|
| `u`     | Eingangsspannung     | V        |
| `i`     | Ankerstrom           | A        |
| `omega` | Winkelgeschwindigkeit| rad/s    |
| `R`     | Ankerwiderstand      | Ω        |
| `L`     | Ankerinduktivität    | H        |
| `K_e`   | Gegen-EMK-Konstante  | V·s/rad  |
| `K_t`   | Drehmomentkonstante  | N·m/A    |
| `J`     | Trägheitsmoment      | kg·m²    |
| `b`     | Reibungskoeffizient  | N·m·s/rad|

## Ausführung

```bash
python labor1_modellierung.py
```

Das Skript:
1. Definiert die Modellparameter des DC-Motors
2. Leitet das Zustandsraummodell und die Übertragungsfunktion her
3. Simuliert die Sprungantwort des Systems
4. Speichert die Modellparameter in `data/modell_parameter.json`
   (wird von Labor 2 und Labor 3 benötigt)

## Ausgabe
- Plot der Sprungantwort
- `data/modell_parameter.json` mit den Modellparametern
