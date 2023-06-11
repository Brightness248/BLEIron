from machine import Pin,Timer
import machine
from time import sleep_ms
import ubluetooth
from random import randint
import time
from max6675 import MAX6675
#import neopixel
from neopixel import NeoPixel
import binascii
neo = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
 
 
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
        for x in neo:
            pixels[x] = (0 , 255 , 0)
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
        for x in neo:
            pixels[x] = (0 , 0 , 255)
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
            print(buffer)
            print(str(message))
            
    def register(self):        
        # Nordic UART Service (NUS)
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
            
        BLE_NUS = ubluetooth.UUID(NUS_UUID)
        BLE_RX = (ubluetooth.UUID(RX_UUID), ubluetooth.FLAG_WRITE)
        BLE_TX = (ubluetooth.UUID(TX_UUID), ubluetooth.FLAG_NOTIFY | ubluetooth.FLAG_READ)
        #BLE_TX = (ubluetooth.UUID(TX_UUID), ubluetooth.FLAG_READ)
        
        BLE_UART = (BLE_NUS, (BLE_TX, BLE_RX,))
        SERVICES = (BLE_UART, )
        ((self.tx, self.rx,), ) = self.ble.gatts_register_services(SERVICES)

    def send(self, data):
        self.ble.gatts_notify(0, self.tx, data + '\n')

    def advertiser(self):
        name = bytes(self.name, 'UTF-8')
        adv_data = bytearray('\x02\x01\x02') + bytearray((len(name) + 1, 0x09)) + name
        self.ble.gap_advertise(100, adv_data)
        print(adv_data)
        print("\r\n")

   
def controlT(Tsoll, Tist):
    Tsoll = int(Tsoll)
    Tist = int(Tist)
    if Tist < Tsoll:
        ssr.duty(int(1023/100*25))
        for x in neo:
            pixels[x] = (255 , 0 , 0)
            pixels.write()
    else:
        ssr.duty(0)
        for x in neo:
            pixels[x] = (0 , 255 , 0)
            pixels.write()

buz = machine.Pin(4,machine.Pin.OUT)

ssrpin = Pin(18,Pin.OUT)
ssr = machine.PWM(ssrpin, freq=1)
ssrpin.value(0)
ssr.duty(0)

fan = Pin(2,Pin.OUT)
bel = Pin(17,Pin.OUT)
pixels = NeoPixel(Pin(15), 16)
ble = ESP32_BLE("ESP32")

so = Pin(19, Pin.IN)
cs = Pin(23, Pin.OUT)
sck = Pin(5, Pin.OUT)
max = MAX6675()

status = 0
Traum = max.readCelsius()
print(Traum)
T = Traum
realT = Traum
ctimeT = 0

sleep_ms(300)

while True:
    print
    if status == 0:
        if time.time()%2 == 0:
            realT = max.readCelsius()
            try:            
                ble.send(str(round(realT, 1)))
            except:
                continue
    if message == "A1":
        status = 0
        print("Abbruch", status)
        message=""
        fan.value(1)
        ssr.duty(0)
        print("Lüfter einschalten")
        #ble.send(str('Lüfter ein und aus die Maus!'))
        buzzer = machine.PWM(buz)
        buzzer.freq(2000)
        buzzer.duty(50)
        sleep_ms(100)
        buzzer.duty(0)
        buzzer.deinit()
        for x in neo:
            pixels[x] = (0 , 255 , 0)
            pixels.write()
    elif 'time' in message:
        msplit = message.split(',')
        print(msplit)
        Zend = int(msplit[1])
        Tend = int(msplit[3])
        status = 1
        tstart = time.time_ns()
        print("Beginne mit Heizen", status)
        message = ''
    elif message == "B1":
        bel.value(not bel.value())
        print("Beleuchtung toggeln")
        message=""
        buzzer = machine.PWM(buz)
        buzzer.freq(2000)
        buzzer.duty(50)
        sleep_ms(100)
        buzzer.duty(0)
        buzzer.deinit()
    elif message == "K1":
        fan.value(not fan.value())
        print("Lüfter toggeln")
        message=""
        buzzer = machine.PWM(buz)
        buzzer.freq(2000)
        buzzer.duty(50)
        sleep_ms(100)
        buzzer.duty(0)
        buzzer.deinit()
    elif message == "BR":
        break
    else:
        if status == 0:
            None
        elif status == 1:
            ctime = (time.time_ns() - tstart)/1000000000
            if int(ctime) > ctimeT:
                print('Zeit', round(ctime, 1), 'Soll', round(T, 1), 'Ist', round(realT, 1))
                realT = max.readCelsius()
                # im ersten Viertel von Raumtemperatur auf Endtemperatur minus 30°C heizen
                if ctime <= 4*Zend/10:
                    T = Traum + ctime/(4*Zend/10) * (Tend - Traum - 30)
                    controlT(T, realT)
                    ble.send(str(round(realT, 1)))
                    
                # im zweiten Viertel bei Endtemperatur minus 30°C halten
                elif ctime <= 6*Zend/10:
                    T = Tend - 30
                    controlT(T, realT)
                    ble.send(str(round(realT, 1)))
                
                # im dritten Viertel weiter auf Endtemperatur heizen
                elif ctime <= (6+2)*Zend/10:
                    T = (Tend - 30) + (ctime-(6*Zend/10))/(2*Zend/10) * (Tend - (Tend - 30))
                    controlT(T, realT)
                    ble.send(str(round(realT, 1)))
                    
                # im vierten Viertel auf Endtemperatur halten
                elif ctime <= Zend:
                    T = Tend
                    controlT(T, realT)
                    ble.send(str(round(realT, 1)))
                    
                # danach Lüfter and und aus
                elif ctime > 4*Zend/4:
                    T = 30
                    fan.value(1)
                    ssr.duty(0)
                    print("Lüfter einschalten")
                    #ble.send(str('Lüfter ein und aus die Maus!'))
                    status = 0
                else:
                    None            
            
            ctimeT = int(ctime)
            
