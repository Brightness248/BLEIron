"""
Verbesserte Temperaturregelung mit PID-Regler und Lüftersteuerung
Berücksichtigt thermische Trägheit für bessere Regelung
"""

from machine import Pin, Timer
import machine
from time import sleep_ms, time
import ubluetooth
from neopixel import NeoPixel
from max6675 import MAX6675
import binascii

# ============ EINSTELLUNGEN AUS TRÄGHEITSMESSUNG ============
# Diese Werte nach measure_inertia.py eintragen.
# Beispiel: M,tau,15.2  -> THERMAL_TAU_S = 15.2
# Totzeit grob: Zeit bis messbarer Temperaturanstieg nach Heizer-Start.
THERMAL_TAU_S = 8.0
THERMAL_DEAD_TIME_S = 2.0

# ============ HARDWARE SETUP ============
# Heizplatte (SSR mit PWM)
ssrpin = Pin(18, Pin.OUT)
ssr = machine.PWM(ssrpin, freq=1)
ssrpin.value(0)
ssr.duty(0)

# Lüfter (PWM gesteuert, 60-100%)
fanpin = Pin(2, Pin.OUT)
fan = machine.PWM(fanpin, freq=1000)  # 1 kHz PWM für Lüfter
fan.duty(0)

# Status LEDs
pixels = NeoPixel(Pin(15), 16)
buz = machine.Pin(4, machine.Pin.OUT)

# Sensoren
max_sensor = MAX6675()

# ============ BLE SETUP ============
message = ""

class ESP32_BLE():
    def __init__(self, name):
        self.timer = Timer(0)
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.disconnected()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()

    def connected(self):
        for x in range(16):
            pixels[x] = (0, 255, 0)
            pixels.write()
        
        buzzer = machine.PWM(buz)
        buzzer.freq(1047)
        buzzer.duty(50)
        sleep_ms(116)
        buzzer.duty(0)
        buzzer.freq(3136)
        buzzer.duty(50)
        sleep_ms(93)
        buzzer.duty(0)
        buzzer.freq(2093)
        buzzer.duty(50)
        sleep_ms(709)
        buzzer.duty(0)
        buzzer.deinit()
        self.timer.deinit()

    def disconnected(self):
        ssrpin.value(0)
        ssr.duty(0)
        for x in range(16):
            pixels[x] = (0, 0, 255)
            pixels.write()
        
        buzzer = machine.PWM(buz)
        buzzer.freq(440)
        buzzer.duty(50)
        sleep_ms(233)
        buzzer.duty(0)
        buzzer.freq(349)
        buzzer.duty(50)
        sleep_ms(151)
        buzzer.duty(0)
        buzzer.freq(659)
        buzzer.duty(50)
        sleep_ms(35)
        buzzer.duty(0)
        buzzer.freq(330)
        buzzer.duty(50)
        sleep_ms(151)
        buzzer.duty(0)
        buzzer.deinit()

    def ble_irq(self, event, data):
        global message
        
        if event == 1:
            self.connected()
        elif event == 2:
            self.advertiser()
            self.disconnected()
        elif event == 3:
            buffer = self.ble.gatts_read(self.rx).replace(b'\x00', b'')
            message = buffer.decode('UTF-8').strip()
            print(f"BLE empfangen: {message}")

    def register(self):
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
        
        BLE_NUS = ubluetooth.UUID(NUS_UUID)
        BLE_RX = (ubluetooth.UUID(RX_UUID), ubluetooth.FLAG_WRITE)
        BLE_TX = (ubluetooth.UUID(TX_UUID), ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ)
        
        BLE_UART = (BLE_NUS, (BLE_TX, BLE_RX,))
        SERVICES = (BLE_UART, )
        ((self.tx, self.rx,), ) = self.ble.gatts_register_services(SERVICES)

    def send(self, data):
        try:
            self.ble.gatts_notify(0, self.tx, data + '\n')
        except:
            pass

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        adv_data = bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name
        self.ble.gap_advertise(100, adv_data)

# ============ PID REGLER ============
class PIDController:
    """Vereinfachter PID-Regler mit Anti-Windup"""
    
    def __init__(self, kp=0.8, ki=0.05, kd=0.3, output_min=0, output_max=100):
        """
        Args:
            kp: Proportionalfaktor (typisch 0.5-2.0)
            ki: Integralfaktor (typisch 0.02-0.2)
            kd: Differentialfaktor (typisch 0.1-0.5)
            output_min, output_max: Ausgabegrenzen (0-100%)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral = 0
        self.last_error = 0
        self.last_time = time()

    def update(self, setpoint, measured_value, dt=None):
        """
        Berechnet den PID-Ausgabewert
        
        Args:
            setpoint: Sollwert (Zieltemperatur)
            measured_value: Istwert (aktuelle Temperatur)
            dt: Zeitdifferenz (automatisch berechnet, wenn None)
        
        Returns:
            Heizleistung in % (0-100)
        """
        current_time = time()
        
        if dt is None:
            dt = current_time - self.last_time
            if dt <= 0:
                dt = 0.1
        
        self.last_time = current_time
        
        # Regelabweichung
        error = setpoint - measured_value
        
        # P-Anteil
        p_term = self.kp * error
        
        # I-Anteil (mit Anti-Windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, 10), -10)  # Limit Integral
        i_term = self.ki * self.integral
        
        # D-Anteil (nur auf Messwert, nicht auf Sollwert)
        d_term = self.kd * (self.last_error - error) / dt if dt > 0 else 0
        self.last_error = error
        
        # Gesamtausgang
        output = p_term + i_term + d_term
        
        # Limitieren
        output = max(min(output, self.output_max), self.output_min)
        
        return output

    def reset(self):
        """Setzt den Regler zurück"""
        self.integral = 0
        self.last_error = 0
        self.last_time = time()

# ============ LÜFTERSTEUERUNG ============
class FanController:
    """Intelligente Lüftersteuerung für 12V Lüfter (60-100% PWM)"""
    
    MIN_PWM = 60  # Mindestens 60% für Anlauf
    MAX_PWM = 100
    TEMP_THRESHOLD = 10  # °C über Sollwert für Lüfter einschalten
    
    def __init__(self):
        self.current_pwm = 0
        self.fan_on = False

    def update(self, setpoint, current_temp, overshoot=0):
        """
        Berechnet die Lüfter-PWM basierend auf Temperaturregelung
        
        Args:
            setpoint: Sollwert (Zieltemperatur)
            current_temp: Aktuelle Temperatur
            overshoot: Bereits gemessenes Überschwingen (°C)
        
        Returns:
            PWM-Wert (0 oder 60-100)
        """
        # Lüfter einschalten, wenn Temperatur zu hoch
        if current_temp >= setpoint + self.TEMP_THRESHOLD:
            # PWM berechnen basierend auf Übertemperatur
            excess = current_temp - setpoint
            pwm = min(self.MIN_PWM + (excess * 3), self.MAX_PWM)
            self.current_pwm = pwm
            self.fan_on = True
        else:
            # Lüfter ausschalten wenn Temperatur wieder normal
            if current_temp <= setpoint + 2:  # Hysterese
                self.current_pwm = 0
                self.fan_on = False
        
        return self.current_pwm

# ============ TEMPERATURREGELUNG ============
class TemperatureController:
    """Hauptregelungslogik"""
    
    def __init__(self):
        self.pid = PIDController(kp=1.0, ki=0.08, kd=0.4)
        self.fan = FanController()
        self.last_send_time = 0
        self.send_interval = 0.5  # Daten jede 0.5s senden
        self.last_temp = None
        self.last_temp_time = None
        
        # Thermische Trägheit (aus Messungen)
        # Diese können nach Durchführung von measure_inertia.py angepasst werden
        self.tau = THERMAL_TAU_S  # Zeitkonstante in Sekunden
        self.dead_time = THERMAL_DEAD_TIME_S  # Totzeit in Sekunden
        
    def update(self, setpoint, current_temp):
        """
        Hauptregelungsschritt
        
        Returns:
            (heater_pwm, fan_pwm, status_text)
        """
        # Temperaturanstieg berechnen (dT/dt)
        now = time()
        temp_rate = 0.0
        if self.last_temp is not None and self.last_temp_time is not None:
            dt = now - self.last_temp_time
            if dt > 0:
                temp_rate = (current_temp - self.last_temp) / dt

        self.last_temp = current_temp
        self.last_temp_time = now

        # Vorausschauende Temperatur: berücksichtigt die Totzeit.
        # Bei starkem Anstieg wird früher Leistung reduziert.
        predicted_temp = current_temp + (temp_rate * self.dead_time)
        control_temp = max(current_temp, predicted_temp)

        # PID-Regler mit vorausschauender Temperatur
        heater_output = self.pid.update(setpoint, control_temp)
        
        # Lüfter berechnen
        fan_setpoint = setpoint - max(0.0, temp_rate * self.tau * 0.15)
        fan_output = self.fan.update(fan_setpoint, current_temp)
        
        # Status-Text
        error = setpoint - current_temp
        status = (
            f"T_ist={current_temp:.1f}°C, T_pred={control_temp:.1f}°C, "
            f"T_soll={setpoint}°C, Fehler={error:+.1f}°C"
        )
        
        # Heizer-PWM anpassen (minimal 15% für Grundlast, maximal 100%)
        if abs(error) < 0.5:
            heater_pwm = 0  # Bei Zielwert abschalten
        elif error > 0:
            heater_pwm = max(15, min(heater_output, 100))
        else:
            heater_pwm = max(0, heater_output)
        
        return heater_pwm, fan_output, status

    def set_thermal_params(self, tau, dead_time):
        """Setzt thermische Parameter (aus Messung)"""
        self.tau = tau
        self.dead_time = dead_time
        print(f"Thermische Parameter: τ={tau}s, Totzeit={dead_time}s")

# ============ HAUPTPROGRAMM ============

# Hardware initialisieren
ble = ESP32_BLE("ESP32")
controller = TemperatureController()

status = 0  # 0=Idle, 1=Heizen
message = ""
last_print_time = time()

print("\n" + "="*70)
print("VERBESSERTE TEMPERATURREGELUNG")
print("="*70)
print("System läuft...")

sleep_ms(500)

# Hauptschleife
while True:
    try:
        # Temperatur auslesen
        current_temp = max_sensor.readCelsius()
        current_time = time()
        
        # Status 0: Standby
        if status == 0:
            controller.pid.reset()
            ssr.duty(0)
            fan.duty(0)
            
            # Temperaturdaten senden (jede Sekunde)
            if current_time - last_print_time > 1.0:
                try:
                    ble.send(f"Idle: {current_temp:.1f}°C")
                except:
                    pass
                last_print_time = current_time
        
        # Status 1: Heizen
        elif status == 1:
            heater_pwm, fan_pwm, status_text = controller.update(target_temp, current_temp)
            
            # Heizleistung setzen
            ssr.duty(int(1023 / 100 * heater_pwm))
            
            # Lüfter setzen
            if fan_pwm > 0:
                fan.duty(int(1023 / 100 * fan_pwm))
            else:
                fan.duty(0)
            
            # Daten senden
            if current_time - last_print_time > controller.send_interval:
                data_str = f"H:{int(heater_pwm)},F:{int(fan_pwm)},T:{current_temp:.1f}"
                try:
                    ble.send(data_str)
                except:
                    pass
                
                # Debug-Ausgabe
                if current_time - last_print_time > 2.0:
                    print(f"{status_text} | Heizer={int(heater_pwm)}% Lüfter={int(fan_pwm)}%")
                    last_print_time = current_time
            
            # Zieltemperatur erreicht?
            if abs(current_temp - target_temp) < 1.0:
                led_r = 0
                led_g = 255
                led_b = 0
            else:
                # LED rot wenn zu kalt, gelb wenn zu heiß
                if current_temp < target_temp:
                    led_r = 255
                    led_g = 0
                    led_b = 0
                else:
                    led_r = 255
                    led_g = 255
                    led_b = 0
            
            for x in range(16):
                pixels[x] = (led_r, led_g, led_b)
            pixels.write()
        
        # BLE-Befehle verarbeiten
        if message == "A1":
            # Abbruch
            status = 0
            message = ""
            fan.duty(0)
            ssr.duty(0)
            print("Regelung abgebrochen")
            
            buzzer = machine.PWM(buz)
            buzzer.freq(2000)
            buzzer.duty(50)
            sleep_ms(100)
            buzzer.duty(0)
            buzzer.deinit()
            
        elif 'time' in message or 'temp' in message:
            # Format: "time,<duration>,temp,<target>"
            # z.B. "time,600,temp,250"
            try:
                parts = message.split(',')
                target_temp = int(parts[3]) if len(parts) > 3 else 200
                status = 1
                controller.pid.reset()
                print(f"Regelung gestartet: Ziel {target_temp}°C")
                message = ""
                
                buzzer = machine.PWM(buz)
                buzzer.freq(2093)
                buzzer.duty(50)
                sleep_ms(100)
                buzzer.duty(0)
                buzzer.deinit()
            except:
                message = ""
        
        elif message == "B1":
            # Beleuchtung (nicht implementiert in dieser Version)
            message = ""
            print("Beleuchtung nicht implementiert")
        
        sleep_ms(50)  # 50ms Regelungsintervall
        
    except Exception as e:
        print(f"Fehler: {e}")
        ssr.duty(0)
        fan.duty(0)
        sleep_ms(1000)
