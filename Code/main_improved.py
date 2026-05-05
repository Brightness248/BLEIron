"""
Verbesserte Temperaturregelung mit PID-Regler und Lüftersteuerung
Berücksichtigt thermische Trägheit für bessere Regelung
"""

from machine import Pin, Timer
import machine
from time import sleep_ms, time, ticks_ms
import ubluetooth
from neopixel import NeoPixel
from max6675 import MAX6675
import binascii

# ============ EINSTELLUNGEN AUS TRÄGHEITSMESSUNG ============
# Diese Werte nach measure_inertia.py eintragen.
# Beispiel: M,tau,15.2  -> THERMAL_TAU_S = 15.2
# Totzeit grob: Zeit bis messbarer Temperaturanstieg nach Heizer-Start.
THERMAL_TAU_S = 38.0
THERMAL_DEAD_TIME_S = 5.0

# Reflow-Profil und Cooldown
COOLDOWN_END_TEMP_C = 80.0 # Ende der aktiven Lüfterkühlung

# Sensor-Filter gegen kurze Messspruenge durch Stoerungen
TEMP_FILTER_ALPHA = 0.35
MAX_TEMP_STEP_C_PER_S = 6.0

# Reflow-Rampen fuer "Peak bei t = Dauer"
PREHEAT_END_RATIO = 0.55
SOAK_END_RATIO = 0.85
PEAK_REACH_RATIO = 1.00

# ============ HARDWARE SETUP ============
# Heizplatte (SSR mit PWM)
ssrpin = Pin(18, Pin.OUT)
ssr = machine.PWM(ssrpin, freq=1)
ssrpin.value(0)
ssr.duty(0)

# Lüfter (Digital ON/OFF, robuster mit verbautem Lüfter)
fan = Pin(2, Pin.OUT)
fan.value(0)

# Beleuchtung (separater Ausgang)
bel = Pin(17, Pin.OUT)
bel.value(0)

# Status LEDs
pixels = NeoPixel(Pin(15), 16)
buz = machine.Pin(4, machine.Pin.OUT)

# Sensoren
max_sensor = MAX6675()

# ============ BLE SETUP ============
message = ""
SERIAL_DEBUG = False


def log(text):
    """Serielle Debug-Ausgabe zentral ein/aus schalten."""
    if SERIAL_DEBUG:
        print(text)


def send_temp_ble(temp_c):
    """Kompatibel zur bestehenden App: nur Temperatur als Zahl senden."""
    try:
        ble.send(str(round(temp_c, 1)))
    except:
        pass


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def profile_setpoint(start_temp, peak_temp, total_s, elapsed_s):
    """Solltemperatur entlang einer Reflow-Kurve aus der Gesamtdauer berechnen."""
    preheat_t = total_s * PREHEAT_END_RATIO
    soak_t = total_s * SOAK_END_RATIO
    peak_t = total_s * PEAK_REACH_RATIO
    preheat_target = min(150.0, peak_temp - 30.0)
    soak_target = min(peak_temp - 10.0, 170.0)
    if soak_target < preheat_target:
        soak_target = preheat_target + 5.0

    if elapsed_s <= preheat_t:
        frac = elapsed_s / preheat_t if preheat_t > 0 else 1.0
        return start_temp + frac * (preheat_target - start_temp)

    if elapsed_s <= soak_t:
        frac = (elapsed_s - preheat_t) / max(0.1, (soak_t - preheat_t))
        return preheat_target + frac * (soak_target - preheat_target)

    if elapsed_s <= peak_t:
        frac = (elapsed_s - soak_t) / max(0.1, (peak_t - soak_t))
        return soak_target + frac * (peak_temp - soak_target)

    return peak_temp


def pulse_green_leds():
    """Lässt alle LEDs grün pulsieren (Cooldown-Phase)."""
    phase = (ticks_ms() // 40) % 100
    if phase < 50:
        level = int(255 * phase / 50)
    else:
        level = int(255 * (100 - phase) / 50)

    for x in range(16):
        pixels[x] = (0, level, 0)
    pixels.write()

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
            log(f"BLE empfangen: {message}")

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
        # Konservativer abgestimmt, um Ueberschwingen zu reduzieren
        self.pid = PIDController(kp=0.75, ki=0.03, kd=0.45)
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
        
        # Status-Text (ASCII-only for robust parsing on device)
        error = setpoint - current_temp
        status = f"T_ist={current_temp:.1f}degC, T_pred={control_temp:.1f}degC, T_soll={setpoint}degC, Fehler={error:+.1f}degC"
        
        # Heizer-PWM anpassen: keine feste Mindestleistung (15%) mehr,
        # damit niedrige Sollwerte nicht ueberschossen werden.
        if error <= 0:
            heater_pwm = 0
        elif error < 1.5:
            heater_pwm = min(max(heater_output, 0), 35)
        else:
            heater_pwm = min(max(heater_output, 0), 100)

        # Vorausschauender Cutoff kurz vor Sollwert, wenn Temperatur noch steigt.
        if current_temp >= (setpoint - 0.4) and temp_rate > 0:
            heater_pwm = 0
        
        return heater_pwm, fan_output, status

    def set_thermal_params(self, tau, dead_time):
        """Setzt thermische Parameter (aus Messung)"""
        self.tau = tau
        self.dead_time = dead_time
        log(f"Thermische Parameter: τ={tau}s, Totzeit={dead_time}s")

# ============ HAUPTPROGRAMM ============

# Hardware initialisieren
ble = ESP32_BLE("ESP32")
controller = TemperatureController()

status = 0  # 0=Idle, 1=Heizen
message = ""
manual_fan_override = False
manual_fan_on = False
process_duration_s = 0
profile_peak_temp = 200
profile_start_temp = 25.0
process_start_time = 0
peak_hold_s = 12.0
peak_hold_start = None
cooldown_active = False
last_print_time = time()
last_debug_time = time()
last_sensor_read_time = 0
current_temp = 25.0  # Startwert
raw_temp = 25.0

log("\n" + "="*70)
log("VERBESSERTE TEMPERATURREGELUNG")
log("="*70)
log("System läuft...")

sleep_ms(500)

# Hauptschleife
while True:
    try:
        current_time = time()

        # MAX6675 max. 1x pro Sekunde lesen (sonst Rauschspikes vom SSR)
        if current_time - last_sensor_read_time >= 1.0:
            new_raw_temp = max_sensor.readCelsius()

            # Erster Messwert: direkt uebernehmen
            if last_sensor_read_time == 0:
                current_temp = new_raw_temp
            else:
                sample_dt = max(0.5, current_time - last_sensor_read_time)
                max_step = MAX_TEMP_STEP_C_PER_S * sample_dt

                # Unplausible Einzelspitzen begrenzen
                delta = new_raw_temp - current_temp
                if delta > max_step:
                    new_raw_temp = current_temp + max_step
                elif delta < -max_step:
                    new_raw_temp = current_temp - max_step

                # Glaettung fuer stabile Kurve in der App
                current_temp = current_temp + TEMP_FILTER_ALPHA * (new_raw_temp - current_temp)

            raw_temp = new_raw_temp
            last_sensor_read_time = current_time
        
        # Status 0: Standby
        if status == 0:
            controller.pid.reset()
            ssr.duty(0)
            if manual_fan_override:
                fan.value(1 if manual_fan_on else 0)
            else:
                fan.value(0)
            
            # Temperaturdaten senden (jede Sekunde)
            if current_time - last_print_time > 1.0:
                send_temp_ble(current_temp)
                last_print_time = current_time
        
        # Status 1: Heizen
        elif status == 1:
            elapsed = current_time - process_start_time
            ramp_elapsed = min(elapsed, process_duration_s)
            dynamic_setpoint = profile_setpoint(
                profile_start_temp,
                profile_peak_temp,
                process_duration_s,
                ramp_elapsed
            )

            # Peak-Hold startet erst nach Ablauf der Soll-Zeit (time-to-peak)
            if elapsed >= process_duration_s:
                if peak_hold_start is None:
                    peak_hold_start = current_time
                dynamic_setpoint = profile_peak_temp

            hold_done = (
                peak_hold_start is not None and
                (current_time - peak_hold_start) >= peak_hold_s
            )

            if hold_done:
                cooldown_active = True
                status = 0
                ssr.duty(0)
                fan.value(1)
                manual_fan_override = True
                manual_fan_on = True
                pulse_green_leds()
                message = ""
                continue

            heater_pwm, fan_pwm, status_text = controller.update(dynamic_setpoint, current_temp)
            
            # Heizleistung setzen
            ssr.duty(int(1023 / 100 * heater_pwm))
            
            # Lüfter setzen
            if manual_fan_override:
                fan.value(1 if manual_fan_on else 0)
            else:
                fan.value(1 if fan_pwm > 0 else 0)
            
            # Daten senden
            if current_time - last_print_time > controller.send_interval:
                send_temp_ble(current_temp)
                last_print_time = current_time
                
                # Debug output (ASCII-only)
                if current_time - last_debug_time > 2.0:
                    log(f"{status_text} | Soll_dyn={dynamic_setpoint:.1f}degC | Heizer={int(heater_pwm)}% Luefter={int(fan_pwm)}%")
                    last_debug_time = current_time
            
            # Zieltemperatur erreicht?
            if abs(current_temp - dynamic_setpoint) < 1.0:
                led_r = 0
                led_g = 255
                led_b = 0
            else:
                # LED rot wenn zu kalt, gelb wenn zu heiß
                if current_temp < dynamic_setpoint:
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

        # Cooldown-Phase nach Reflow-Ende
        if cooldown_active:
            ssr.duty(0)
            fan.value(1)
            pulse_green_leds()

            if current_time - last_print_time > 1.0:
                send_temp_ble(current_temp)
                last_print_time = current_time

            # Bei ausreichend Abkühlung in normalen Idle zurück
            if current_temp <= COOLDOWN_END_TEMP_C:
                cooldown_active = False
                manual_fan_override = False
                manual_fan_on = False
                fan.value(0)
        
        # BLE-Befehle verarbeiten
        if message == "A1":
            # Abbruch
            status = 0
            message = ""
            cooldown_active = False
            manual_fan_override = True
            manual_fan_on = True
            fan.value(1)
            ssr.duty(0)
            peak_hold_start = None
            log("Regelung abgebrochen")
            
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
                process_duration_s = int(parts[1]) if len(parts) > 1 else 240
                profile_peak_temp = int(parts[3]) if len(parts) > 3 else 230
                process_duration_s = max(60, process_duration_s)
                # Zieltemperatur aus App respektieren (z. B. 120C Testfahrten)
                profile_peak_temp = int(clamp(profile_peak_temp, 50, 260))

                peak_hold_s = clamp(process_duration_s * 0.06, 8.0, 20.0)
                profile_start_temp = current_temp
                process_start_time = current_time
                peak_hold_start = None

                status = 1
                cooldown_active = False
                manual_fan_override = False
                manual_fan_on = False
                controller.pid.reset()
                log("Reflow gestartet: Dauer=%ss, Peak=%sdegC, Hold=%.1fs" % (process_duration_s, profile_peak_temp, peak_hold_s))
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
            # Beleuchtung toggeln (kompatibel zu main.py)
            bel.value(0 if bel.value() else 1)
            message = ""
            log("Beleuchtung toggeln")

            buzzer = machine.PWM(buz)
            buzzer.freq(2000)
            buzzer.duty(50)
            sleep_ms(100)
            buzzer.duty(0)
            buzzer.deinit()

        elif message == "K1":
            # Lüfter toggeln (manuelle Übersteuerung)
            cooldown_active = False
            manual_fan_override = True
            manual_fan_on = not manual_fan_on
            fan.value(1 if manual_fan_on else 0)
            message = ""
            log("Luefter toggeln")

            buzzer = machine.PWM(buz)
            buzzer.freq(2000)
            buzzer.duty(50)
            sleep_ms(100)
            buzzer.duty(0)
            buzzer.deinit()

        elif message == "K0":
            # Lüfter explizit AUS (manuell)
            cooldown_active = False
            manual_fan_override = True
            manual_fan_on = False
            fan.value(0)
            message = ""
            log("Luefter aus (manuell)")

        elif message == "KA":
            # Lüfter zurück auf Automatik
            cooldown_active = False
            manual_fan_override = False
            message = ""
            log("Luefter auf Automatik")

        elif message == "BR":
            # Kompatibel zu main.py: Schleife verlassen
            break
        
        sleep_ms(50)  # 50ms Regelungsintervall
        
    except Exception as e:
        log(f"Fehler: {e}")
        ssr.duty(0)
        fan.value(0)
        sleep_ms(1000)
