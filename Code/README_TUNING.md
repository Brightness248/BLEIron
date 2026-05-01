# Temperaturregelung mit Traegheitsmessung (ESP32 + MicroPython)

## Ziel
Diese Anleitung beschreibt kurz:
1. Wie du die thermische Traegheit misst
2. Welche Werte du aus BLE brauchst
3. Wo du die Werte in der Regelung eintraegst

## Dateien
- Messung: measure_inertia.py
- Regelung: main_improved.py

## 1) Traegheit messen
1. `measure_inertia.py` auf den ESP32 laden und starten.
2. Das Skript faehrt automatisch 3 Tests (50%, 75%, 100% Heizleistung).
3. Sicherheitsregel: Es wird maximal bis 200 Grad C getestet (harte Abschaltung bei 200 Grad C).
3. Alle wichtigen Daten werden per BLE gesendet.

## 2) Relevante BLE-Nachrichten
Wichtige Zeilen fuer das Tuning:

- `M,tau,15.2`
  - Das ist direkt die Zeitkonstante `tau` in Sekunden.

- `M,result,start,24.8,target,200.0,max,206.1,ov,6.1,tt,45.1,tm,48.4`
  - `ov`: Overshoot in Grad C
  - `tt`: Zeit bis Zieltemperatur
  - `tm`: Zeit bis Temperaturmaximum

- `M,start,target,200,power,50,t0,24.8`
  - Startpunkt der jeweiligen Messung

- `M,live,t,12.3,temp,85.5,dT,60.7,max,85.5`
  - Live-Verlauf; nutzbar um die Totzeit grob abzuschaetzen

## 3) Werte in die Regelung eintragen
In `main_improved.py` direkt oben in der Konfigurationssektion:

```python
THERMAL_TAU_S = 8.0
THERMAL_DEAD_TIME_S = 2.0
```

Ersetze:
- `THERMAL_TAU_S` mit deinem Wert aus `M,tau,...`
- `THERMAL_DEAD_TIME_S` mit der geschaetzten Totzeit

## Totzeit praktisch bestimmen
Wenn keine direkte Totzeit ausgegeben wird:
1. Nimm den Zeitpunkt von `M,start,...` als `t0`.
2. Suche in `M,live,...` den ersten klaren Temperaturanstieg (z. B. +0.5 bis +1.0 Grad C).
3. `Totzeit = t_anstieg - t0`.

## Wie die Werte genutzt werden
`main_improved.py` verwendet die Werte so:
- `tau`: beeinflusst das fruehere Kuehlen/Luefterverhalten
- `dead_time`: erzeugt eine vorausschauende Temperatur (`predicted_temp`), damit der Heizer frueher reduziert wird

Damit wird Ueberschwingen reduziert und die Zieltemperatur besser getroffen.

## Kurz-Check nach dem Eintragen
1. Regelung starten (wie bisher per BLE-Startkommando).
2. Beobachte Live-Werte `H:...,F:...,T:...`.
3. Bei zu viel Overshoot:
   - `THERMAL_DEAD_TIME_S` leicht erhoehen
   - oder `kd` leicht erhoehen
4. Bei zu traeger Reaktion:
   - `THERMAL_DEAD_TIME_S` leicht senken
   - oder `kp` leicht erhoehen

## Hinweise zum Luefter
Der Luefter bleibt im sicheren Bereich fuer deinen 12V-Luefter:
- PWM nur 60-100% (oder aus)
- Kein Dauerbetrieb unter 50%

## Schnelldiagnose bei dT = 0
Wenn die Traegheitsmessung keine Temperaturaenderung zeigt, nutze `diag_heater_path.py`.

### Ablauf
1. `diag_heater_path.py` auf den ESP32 laden und starten.
2. BLE-Log beobachten.
3. Wichtige Meldungen:
   - `D,result,base,...` -> Sensor-Basischeck
   - `D,result,p30,delta,...`
   - `D,result,p60,delta,...`
   - `D,result,p100,delta,...`
   - `D,conclusion,...`

### Interpretation
- `D,conclusion,path_ok`
  - Heizpfad und Sensor reagieren grundsaetzlich.
- `D,conclusion,heater_path_fault`
  - Sensor lebt, aber Heizwirkung fehlt. SSR/Netzpfad/Heizelement pruefen.
- `D,conclusion,sensor_or_wiring_fault`
  - Sensorleitung oder Sensor selbst pruefen (MAX6675, Thermoelement-Polaritaet, Kontakt).
