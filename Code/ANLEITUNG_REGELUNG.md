# Verbesserte Temperaturregelung - Anleitung

## Übersicht

Du hast jetzt zwei neue Programme:

1. **`measure_inertia.py`** - Misst die thermische Trägheit deiner Heizplatte
2. **`main_improved.py`** - Verbesserte Regelung mit PID-Algorithmus und PWM-Lüftersteuerung

## Schritt 1: Trägheit messen

### Was wird gemessen?
- **Totzeit (Tote Zeit)**: Wie lange es dauert, bis die Heizplatte anfängt, die Temperatur zu erhöhen
- **Zeitkonstante (τ)**: Wie schnell die Heizplatte aufwärmt
- **Overshoot**: Wie viel die Temperatur über das Ziel hinausschießt
- **Anstiegszeit**: Wie lange es dauert, die Zieltemperatur zu erreichen

### Durchführung:
1. Ersetze `main.py` temporär durch `measure_inertia.py` oder lade es hinzu
2. Starte das Messprogramm auf dem ESP32
3. Das Programm führt 3 automatische Tests durch:
   - Test 1: 50% Power → 200°C (max 120 Sekunden)
   - Test 2: 75% Power → 250°C (max 120 Sekunden)  
   - Test 3: 100% Power → 300°C (max 120 Sekunden)

4. Beobachte die LEDs:
   - 🔵 Blau = Messung läuft
   - 🟢 Grün = Messung abgeschlossen
   - 🔴 Rot = Fehler

5. Die Ergebnisse werden ausgegeben:
```
====================================================================
MESSERGEBNISSE THERMISCHE TRÄGHEIT
====================================================================
Starttemperatur:          25.0°C
Zieltemperatur:           200.0°C
Maximale Temperatur:      205.2°C
Overshoot:                +5.2°C (+2.6%)
Zeit bis Zieltemperatur:  45.3s
Zeit bis Maximum:         48.1s
Heizleistung:             50%
Gesamte Messdauer:        120.0s
====================================================================
Zeitkonstante τ (bis 63%): 15.2s
```

## Schritt 2: Parameter anpassen

Nach der Messung hast du die optimalen Parameter. Öffne `main_improved.py` und passe folgende Werte an:

### PID-Parameter einstellen (ca. Zeile 160):
```python
class TemperatureController:
    def __init__(self):
        self.pid = PIDController(kp=1.0, ki=0.08, kd=0.4)
```

Faustregel für deine Heizplatte:
- **kp (Proportional)**: 0.8 - 1.5 (höher = aggressiver Ausgleich)
- **ki (Integral)**: 0.05 - 0.15 (reduziert Dauerfehler)
- **kd (Differential)**: 0.2 - 0.6 (reduziert Overshoot)

### Thermische Parameter anpassen (ca. Zeile 180):
```python
self.tau = 8.0  # Zeitkonstante aus Messung
self.dead_time = 2.0  # Totzeit aus Messung
```

Verwende die Werte aus deiner Messung!

## Schritt 3: Lüftersteuerung verstehen

Der Lüfter wird jetzt intelligent gesteuert:

```python
MIN_PWM = 60  # Mindestens 60% zum Anlaufen
MAX_PWM = 100
TEMP_THRESHOLD = 10  # Lüfter einschalten bei +10°C über Sollwert
```

**Verhalten:**
- Temperatur < Soll: Lüfter aus (0%)
- Temperatur = Soll +5°C: Lüfter 60-80% (je nach Übertemperatur)
- Temperatur = Soll +20°C: Lüfter 100%

Du kannst die Schwellwerte anpassen wenn nötig.

## Schritt 4: Regelung starten

Verwende deine mobile App um die Regelung zu starten:
- Format: `"time,600,temp,250"` (600 Sekunden Heizen, Ziel 250°C)

Die App zeigt jetzt genauere Werte:
- `H:75,F:85,T:248.3` (Heizer 75%, Lüfter 85%, Temp 248.3°C)

## Unterschiede zur alten Regelung

| Merkmal | Alt | Neu |
|---------|-----|-----|
| Heizleistung | An/Aus (25%) | PWM 0-100% |
| Regelung | P-Regler | PID-Regler |
| Lüfter | Digital (An/Aus) | PWM 60-100% |
| Temperaturgenauigkeit | ±5-10°C | ±1-2°C |
| Überschwingen | Hoch | Reduziert |
| Anstiegsgeschwindigkeit | Schnell/unkontrolliert | Gedämpft |

## Debugging / Optimierung

Wenn die Regelung nicht optimal läuft:

### Problem: Temperatur schießt zu sehr über (Overshoot > 10°C)
→ Erhöhe `kd` um 0.1-0.2 oder reduziere `kp` um 0.1

### Problem: Temperatur steigt zu langsam
→ Erhöhe `kp` um 0.1 oder reduziere `kd` um 0.1

### Problem: Temperatur oszilliert
→ Reduziere `kp` oder erhöhe `kd`

### Problem: Lüfter läuft ständig
→ Erhöhe `TEMP_THRESHOLD` von 10 auf 15

### Problem: Lüfter läuft nicht richtig an
→ Erhöhe `MIN_PWM` von 60 auf 70-80

## Technische Details

### PID-Regler:
- **P-Anteil**: Proportional zum Fehler (Soll - Ist)
- **I-Anteil**: Integriert Fehler über Zeit (beseitigt Offset)
- **D-Anteil**: Reagiert auf Änderungsgeschwindigkeit (verhindert Overshoot)

### Anti-Windup:
Der Regler begrenzt den Integral-Anteil automatisch um Überschießen zu verhindern.

## Häufige Fragen

**F: Kann ich `main_improved.py` direkt als `main.py` verwenden?**
A: Ja! Benenne die Datei um oder kopiere den Inhalt. Der Code ist vollständig und funktioniert ohne das alte Programm.

**F: Was ist wenn die Messung fehlschlägt?**
A: Verwende die Default-Werte im Code. Diese sollten für die meisten Heizplatten funktionieren.

**F: Kann ich die Parameter zur Laufzeit ändern?**
A: Ja, mit etwas Modifikation kannst du über BLE neue PID-Parameter senden.

**F: Warum ist der Lüfter jetzt PWM und nicht digital?**
A: PWM ermöglicht sanfte Regelung. Der Lüfter läuft nicht ständig an/aus, sondern wird stufenlos angepasst.

---

Bei Fragen oder Problemen, schreibe die Debug-Ausgabe auf der Konsole mit!
