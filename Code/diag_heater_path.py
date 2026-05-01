"""
Diagnose fuer Heizpfad und Sensorreaktion (ESP32 + MAX6675)
Sendet alle Ergebnisse ueber BLE (NUS), damit kein Serial noetig ist.

Ablauf:
1) Sensor-Basischeck (10 s, Heizer AUS)
2) Heizstufen-Test (30%, 60%, 100%; je 20 s)
3) Auswertung und Empfehlung
"""

from machine import Pin
import machine
from time import sleep_ms, ticks_ms, ticks_diff
import ubluetooth
from max6675 import MAX6675

SAMPLE_INTERVAL_MS = 1000

# Hardware
ssrpin = Pin(18, Pin.OUT)
ssr = machine.PWM(ssrpin, freq=1)
ssr.duty(0)

fan = Pin(2, Pin.OUT)
fan.value(0)

buz = machine.Pin(4, machine.Pin.OUT)

# Sensor
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


def set_heater(power_percent):
    power_percent = max(0, min(100, int(power_percent)))
    ssr.duty(int(1023 / 100 * power_percent))
    ble.send("D,heater,{}".format(power_percent))


def short_beep(freq=1800, dur_ms=80):
    buzzer = machine.PWM(buz)
    buzzer.freq(freq)
    buzzer.duty(50)
    sleep_ms(dur_ms)
    buzzer.duty(0)
    buzzer.deinit()


def sample_temp(seconds, heater_percent, label):
    set_heater(heater_percent)
    t0 = ticks_ms()
    next_sample = t0
    values = []

    while ticks_diff(ticks_ms(), t0) < int(seconds * 1000):
        now = ticks_ms()
        if ticks_diff(now, next_sample) >= 0:
            temp = max_sensor.readCelsius()
            rel_t = ticks_diff(now, t0) / 1000.0
            values.append(temp)
            ble.send("D,live,{},t,{:.1f},temp,{:.2f}".format(label, rel_t, temp))
            next_sample = now + SAMPLE_INTERVAL_MS
        sleep_ms(20)

    return values


def analyze(values):
    if not values:
        return 0.0, 0.0, 0.0
    t_min = min(values)
    t_max = max(values)
    delta = t_max - t_min
    return t_min, t_max, delta


def run_diagnostic():
    ble.send("D,ready,1")
    short_beep(2000, 60)

    # Sicherheit: Luefter aus und Heizer aus am Start
    fan.value(0)
    set_heater(0)
    sleep_ms(300)

    # 1) Basischeck: Sensorrauschen / Sensor lebt?
    base = sample_temp(seconds=10, heater_percent=0, label="base")
    bmin, bmax, bdelta = analyze(base)
    ble.send("D,result,base,min,{:.2f},max,{:.2f},delta,{:.2f}".format(bmin, bmax, bdelta))

    # 2) Heizstufen
    p30 = sample_temp(seconds=20, heater_percent=30, label="p30")
    set_heater(0)
    sleep_ms(2000)

    p60 = sample_temp(seconds=20, heater_percent=60, label="p60")
    set_heater(0)
    sleep_ms(2000)

    p100 = sample_temp(seconds=20, heater_percent=100, label="p100")
    set_heater(0)

    p30_min, p30_max, p30_delta = analyze(p30)
    p60_min, p60_max, p60_delta = analyze(p60)
    p100_min, p100_max, p100_delta = analyze(p100)

    ble.send("D,result,p30,delta,{:.2f}".format(p30_delta))
    ble.send("D,result,p60,delta,{:.2f}".format(p60_delta))
    ble.send("D,result,p100,delta,{:.2f}".format(p100_delta))

    # 3) Einfache Diagnose-Regeln
    # - Sensor ok, wenn im Basistest etwas Bewegung oder stabile plausible Werte
    # - Heizpfad ok, wenn bei 60% oder 100% in 20 s wenigstens ~1 C Anstieg sichtbar
    sensor_stuck = bdelta < 0.05
    heater_works = (p60_delta >= 1.0) or (p100_delta >= 1.0)

    if sensor_stuck and not heater_works:
        conclusion = "sensor_or_wiring_fault"
    elif (not sensor_stuck) and (not heater_works):
        conclusion = "heater_path_fault"
    elif sensor_stuck and heater_works:
        conclusion = "unlikely_check_sampling"
    else:
        conclusion = "path_ok"

    ble.send("D,conclusion,{}".format(conclusion))
    ble.send("D,done,1")

    short_beep(2200, 70)
    short_beep(2200, 70)


if __name__ == "__main__":
    try:
        run_diagnostic()
    except Exception as e:
        set_heater(0)
        fan.value(0)
        ble.send("D,error,{}".format(str(e)))
