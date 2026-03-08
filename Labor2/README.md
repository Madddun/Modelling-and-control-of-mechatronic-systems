# Labor 2 – Systemidentifikation

## Ziel
In diesem Labor werden die Parameter des DC-Motor-Modells aus Labor 1 durch
Systemidentifikation geschätzt. Dazu werden Messdaten (Sprungantwort) ausgewertet
und die charakteristischen Modellparameter bestimmt.

## Voraussetzungen
- Labor 1 muss ausgeführt worden sein (`Labor1/data/modell_parameter.json` muss existieren)

## Methode
Es wird eine vereinfachte Systemidentifikation basierend auf der Sprungantwort
des Motors verwendet (Streckenidentifikation, PT1-Näherung oder vollständige
Parameteridentifikation via Least-Squares).

## Ausführung

```bash
python labor2_systemidentifikation.py
```

Das Skript:
1. Lädt die nominalen Parameter aus `../Labor1/data/modell_parameter.json`
2. Generiert synthetische Messdaten mit Rauschen (simuliert reale Messung)
3. Schätzt die Systemparameter mittels Least-Squares-Optimierung
4. Vergleicht identifizierte Parameter mit nominalen Werten
5. Speichert die Ergebnisse in `data/identifizierte_parameter.json`
   (wird von Labor 3 benötigt)

## Ausgabe
- Vergleichsplot: nominales Modell vs. identifiziertes Modell
- `data/identifizierte_parameter.json` mit den identifizierten Parametern
