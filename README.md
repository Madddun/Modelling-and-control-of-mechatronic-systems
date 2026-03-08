# Modelling-and-control-of-mechatronic-systems

Dieses Repository enthält drei aufeinander aufbauende Laboraufgaben zur
**Modellierung und Regelung mechatronischer Systeme** am Beispiel eines DC-Motors.

## Überblick

| Labor  | Thema                     | Eingabe                           | Ausgabe                                        |
|--------|---------------------------|-----------------------------------|------------------------------------------------|
| Labor 1 | Physikalische Modellierung | —                                 | `Labor1/data/modell_parameter.json`            |
| Labor 2 | Systemidentifikation       | Labor 1 Modellparameter           | `Labor2/data/identifizierte_parameter.json`    |
| Labor 3 | Reglerentwurf (PID)        | Labor 1 + Labor 2 Parameter       | `Labor3/data/regler_ergebnisse.json`           |

> **Wichtig:** Labor 3 benötigt die Ausgabedaten aus Labor 1 und Labor 2.
> Bitte die Laboraufgaben in der angegebenen Reihenfolge ausführen.

## Abhängigkeiten

```bash
pip install -r requirements.txt
```

## Ausführung (in Reihenfolge)

```bash
# Schritt 1: Physikalische Modellierung
cd Labor1
python labor1_modellierung.py

# Schritt 2: Systemidentifikation
cd ../Labor2
python labor2_systemidentifikation.py

# Schritt 3: Reglerentwurf
cd ../Labor3
python labor3_reglerentwurf.py
```

## Struktur

```
├── Labor1/
│   ├── README.md
│   ├── labor1_modellierung.py
│   └── data/
│       ├── modell_parameter.json        ← wird von Labor 2 & 3 benötigt
│       └── sprungantwort_modell.png
├── Labor2/
│   ├── README.md
│   ├── labor2_systemidentifikation.py
│   └── data/
│       ├── identifizierte_parameter.json ← wird von Labor 3 benötigt
│       └── systemidentifikation_vergleich.png
├── Labor3/
│   ├── README.md
│   ├── labor3_reglerentwurf.py
│   └── data/
│       ├── regler_ergebnisse.json
│       └── regler_ergebnisse.png
└── requirements.txt
```

## Systemmodell

Als Beispielsystem wird ein **DC-Motor** verwendet:

**Elektrischer Teil:**
```
L · di/dt + R · i = u − K_e · ω
```

**Mechanischer Teil:**
```
J · dω/dt + b · ω = K_t · i
```

**Übertragungsfunktion** G(s) = ω(s)/U(s):
```
         K_t
G(s) = ──────────────────────────────────
        L·J·s² + (R·J + b·L)·s + (R·b + K_e·K_t)
```