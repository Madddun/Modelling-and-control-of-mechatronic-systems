# Labor 3 – Reglerentwurf

## Ziel
In diesem Labor wird ein PID-Regler für den DC-Motor entworfen, dessen Modell in
Labor 1 hergeleitet und in Labor 2 identifiziert wurde.

## Voraussetzungen
- Labor 1 muss ausgeführt worden sein (`Labor1/data/modell_parameter.json` muss existieren)
- Labor 2 muss ausgeführt worden sein (`Labor2/data/identifizierte_parameter.json` muss existieren)

## Entwurfsverfahren
Der PID-Regler wird nach der **Ziegler-Nichols-Methode** und durch
**Phasenrand-Optimierung** im Frequenzbereich entworfen.

## Ausführung

```bash
# Zunächst Labor 1 und 2 ausführen, falls noch nicht geschehen:
cd ../Labor1 && python labor1_modellierung.py
cd ../Labor2 && python labor2_systemidentifikation.py

# Dann Labor 3 ausführen:
cd ../Labor3 && python labor3_reglerentwurf.py
```

Das Skript:
1. Lädt Modellparameter aus `../Labor1/data/modell_parameter.json`
2. Lädt identifizierte Parameter aus `../Labor2/data/identifizierte_parameter.json`
3. Entwirft einen PID-Regler basierend auf dem identifizierten Modell
4. Simuliert den geschlossenen Regelkreis
5. Bewertet die Regelgüte (Überschwingen, Einregelzeit, stationäre Genauigkeit)
6. Speichert die Reglerergebnisse in `data/regler_ergebnisse.json`

## Ausgabe
- Sprungantwort des geregelten Systems
- Bode-Diagramm des offenen Regelkreises
- `data/regler_ergebnisse.json` mit den Reglerparametern und Gütekennzahlen
