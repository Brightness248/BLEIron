"""
Messung der thermischen Trägheit der Heizplatte
Bestimmt: Totzeit, Anstiegszeit, maximale Temperatur, Overshoot
"""

from machine import Pin, Timer
import machine
from time import sleep_ms, time_ns, time
import ubluetooth
from max6675 import MAX6675
from neopixel import NeoPixel
import json

# Hardware Setup
ssrpin = Pin(18, Pin.OUT)
ssr = machine.PWM(ssrpin, freq=1)
ssrpin.value(0)
ssr.duty(0)

fan = Pin(2, Pin.OUT)
fan.value(0)

pixels = NeoPixel(Pin(15), 16)
buz = machine.Pin(4, machine.Pin.OUT)

# Thermoelement
max_sensor = MAX6675()

# BLE
message = ""


class ESP32_BLE:
    def __init__(self, name):
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()

    def ble_irq(self, event, data):
        global message
        if event == 3:
            buffer = self.ble.gatts_read(self.rx).replace(b'\x00', b'')
            message = buffer.decode('UTF-8').strip()

    def register(self):
        nus_uuid = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        rx_uuid = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        tx_uuid = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

        ble_nus = ubluetooth.UUID(nus_uuid)
        ble_rx = (ubluetooth.UUID(rx_uuid), ubluetooth.FLAG_WRITE)
        ble_tx = (ubluetooth.UUID(tx_uuid), ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ)

        ble_uart = (ble_nus, (ble_tx, ble_rx,))
        services = (ble_uart,)
        ((self.tx, self.rx,),) = self.ble.gatts_register_services(services)

    def send(self, data):
        try:
            self.ble.gatts_notify(0, self.tx, data + '\n')
        except:
            pass

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        adv_data = bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name
        self.ble.gap_advertise(100, adv_data)


ble = ESP32_BLE("ESP32")

# Messdaten speichern
measurements = []
start_time = 0
target_temp = 0
max_temp = 0
max_temp_time = 0
target_reached_time = 0
start_temp = 0
heater_power = 0  # PWM Wert in %

def set_heater(power_percent):
    """Setzt die Heizplatte auf einen PWM-Wert (0-100%)"""
    global heater_power
    heater_power = power_percent
    pwm_value = int(1023 / 100 * power_percent)
    ssr.duty(pwm_value)
    print(f"Heizer auf {power_percent}% gesetzt")
    ble.send(f"M,heater,{int(power_percent)}")

def beep(frequency, duration_ms):
    """Macht einen Piepton"""
    buzzer = machine.PWM(buz)
    buzzer.freq(frequency)
    buzzer.duty(50)
    sleep_ms(duration_ms)
    buzzer.duty(0)
    buzzer.deinit()

def led_status(r, g, b):
    """Setzt alle LEDs auf eine Farbe"""
    for x in range(16):
        pixels[x] = (r, g, b)
    pixels.write()

def measure_with_power(target, power_percent, duration_seconds):
    """
    Misst die Temperaturkurve bei konstanter Heizleistung
    
    Args:
        target: Zieltemperatur in °C
        power_percent: Heizleistung in % (0-100)
        duration_seconds: Messdauer in Sekunden
    """
    global start_time, target_temp, max_temp, max_temp_time, target_reached_time
    global start_temp, measurements
    
    measurements = []
    start_time = time()
    target_temp = target
    max_temp = 0
    max_temp_time = 0
    target_reached_time = 0
    
    # Starttemperatur messen
    start_temp = max_sensor.readCelsius()
    print(f"\n=== Messung startet ===")
    print(f"Starttemperatur: {start_temp:.1f}°C")
    print(f"Zieltemperatur: {target}°C")
    print(f"Heizleistung: {power_percent}%")
    print(f"Messdauer: {duration_seconds}s\n")
    ble.send(f"M,start,target,{int(target)},power,{int(power_percent)},t0,{start_temp:.1f}")
    
    # LEDs blau (laufende Messung)
    led_status(0, 0, 255)
    beep(1047, 100)
    
    # Heizer einschalten
    set_heater(power_percent)
    
    # Messdaten sammeln
    while time() - start_time < duration_seconds:
        try:
            current_time = time() - start_time
            temp = max_sensor.readCelsius()
            
            measurements.append({
                'time': current_time,
                'temp': temp
            })
            
            # Maximum tracking
            if temp > max_temp:
                max_temp = temp
                max_temp_time = current_time
            
            # Zieltemperatur erreicht?
            if target_reached_time == 0 and temp >= target:
                target_reached_time = current_time
                print(f"Zieltemperatur erreicht nach {current_time:.1f}s")
                beep(2093, 100)
                ble.send(f"M,target_reached,t,{current_time:.1f},temp,{temp:.1f}")
            
            # Ausgabe jede Sekunde
            if int(current_time) % 1 == 0 and len(measurements) % 3 == 0:
                led_r = int(255 * (temp / 300)) if temp < 300 else 255
                led_g = int(255 * (1 - abs(temp - target) / 100))
                led_b = int(255 * (1 - temp / 300)) if temp < 300 else 0
                led_status(led_r, led_g, led_b)
                print(f"t={current_time:6.1f}s | T={temp:6.1f}°C | ΔT={temp-start_temp:6.1f}°C", end="")
                if temp >= target:
                    print(f" | Overshoot: {max_temp-target:+.1f}°C")
                else:
                    print()
                ble.send(
                    f"M,live,t,{current_time:.1f},temp,{temp:.1f},dT,{(temp-start_temp):.1f},max,{max_temp:.1f}"
                )
            
            sleep_ms(100)  # 100ms Messintervall
            
        except Exception as e:
            print(f"Fehler bei Messung: {e}")
            break
    
    # Heizer ausschalten
    set_heater(0)
    led_status(0, 255, 0)
    beep(2093, 150)
    
    # Ergebnisse auswerten
    print_results()

def print_results():
    """Gibt die Messergebnisse aus"""
    if not measurements:
        print("Keine Messdaten vorhanden!")
        return
    
    overshoot = max_temp - target_temp
    overshoot_percent = (overshoot / target_temp * 100) if target_temp > 0 else 0
    
    print("\n" + "="*70)
    print("MESSERGEBNISSE THERMISCHE TRÄGHEIT")
    print("="*70)
    print(f"Starttemperatur:          {start_temp:.1f}°C")
    print(f"Zieltemperatur:           {target_temp:.1f}°C")
    print(f"Maximale Temperatur:      {max_temp:.1f}°C")
    print(f"Overshoot:                {overshoot:+.1f}°C ({overshoot_percent:+.1f}%)")
    print(f"Zeit bis Zieltemperatur:  {target_reached_time:.1f}s")
    print(f"Zeit bis Maximum:         {max_temp_time:.1f}s")
    print(f"Heizleistung:             {heater_power}%")
    print(f"Gesamte Messdauer:        {measurements[-1]['time']:.1f}s")
    print("="*70 + "\n")
    ble.send(
        f"M,result,start,{start_temp:.1f},target,{target_temp:.1f},max,{max_temp:.1f},ov,{overshoot:.1f},tt,{target_reached_time:.1f},tm,{max_temp_time:.1f}"
    )
    
    # Trägheitskonstante (Zeit zum 63% des Zielwertes) - erste Ordnung
    temp_63 = start_temp + 0.63 * (max_temp - start_temp)
    tau = 0
    for m in measurements:
        if m['temp'] >= temp_63:
            tau = m['time']
            break
    
    if tau > 0:
        print(f"Zeitkonstante τ (bis 63%): {tau:.1f}s")
        ble.send(f"M,tau,{tau:.1f}")
    
    return {
        'start_temp': start_temp,
        'target_temp': target_temp,
        'max_temp': max_temp,
        'overshoot': overshoot,
        'time_to_target': target_reached_time,
        'time_to_max': max_temp_time,
        'power_percent': heater_power,
        'tau': tau
    }

def export_data():
    """Exportiert die Messdaten als JSON"""
    try:
        data = {
            'measurements': measurements,
            'results': {
                'start_temp': start_temp,
                'target_temp': target_temp,
                'max_temp': max_temp,
                'overshoot': max_temp - target_temp,
                'time_to_target': target_reached_time,
                'time_to_max': max_temp_time,
                'power_percent': heater_power
            }
        }
        
        with open('thermal_data.json', 'w') as f:
            json.dump(data, f)
        
        print("Daten gespeichert in thermal_data.json")
        ble.send("M,file,thermal_data.json,saved,1")
    except Exception as e:
        print(f"Fehler beim Speichern: {e}")
        ble.send("M,file,thermal_data.json,saved,0")

# ============ HAUPTPROGRAMM ============
if __name__ == "__main__":
    print("\n" + "="*70)
    print("THERMISCHE TRÄGHEIT - MESSPROGRAMM")
    print("="*70)
    print("Dieses Programm misst die Temperaturkurve der Heizplatte")
    print("und bestimmt optimale Regelungsparameter.\n")
    ble.send("M,ready,1")
    
    try:
        # Test 1: 50% Power bis 200°C
        print("\n>>> TEST 1: 50% Heizleistung, Ziel 200°C (max 120s)")
        measure_with_power(target=200, power_percent=50, duration_seconds=120)
        
        sleep_ms(5000)  # Abkühlung
        
        # Test 2: 75% Power bis 250°C
        print("\n>>> TEST 2: 75% Heizleistung, Ziel 250°C (max 120s)")
        measure_with_power(target=250, power_percent=75, duration_seconds=120)
        
        sleep_ms(5000)  # Abkühlung
        
        # Test 3: 100% Power bis 300°C
        print("\n>>> TEST 3: 100% Heizleistung, Ziel 300°C (max 120s)")
        measure_with_power(target=300, power_percent=100, duration_seconds=120)
        
        # Daten exportieren
        export_data()
        
        print("\n✓ Alle Tests abgeschlossen!")
        ble.send("M,done,1")
        led_status(0, 255, 0)
        beep(2093, 100)
        beep(2093, 100)
        
    except Exception as e:
        print(f"Fehler im Messprogramm: {e}")
        ble.send("M,error,1")
        led_status(255, 0, 0)
        set_heater(0)
        fan.value(0)
