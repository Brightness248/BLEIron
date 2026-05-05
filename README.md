# BLEIron

<p align="center">

  <img height="290" src="/Images/BLEIRON_LOGO.png">
</p>

**BLEIron** is for makers who want to solder not only through-hole but also SMD components. <br>
There are ready-made solutions, but they are usually expensive or too small or both ^^.<br>
Made from an old iron, an SSR, PC power supply, fan and a MAX6675, you can solder your own boards wonderfully. 
With the help of the self-programmed APP, the desired temperature and soldering time are transmitted via Bluetooth Low Energy. Cooling, lighting and a voice output can also be activated. The ACTUAL temperature is displayed in the app. 
The device itself symbolises 3 states via 16 neopixels:<br> 
- blue - BLE device not connected 
- green - BLE connected / ready for operation 
- red - active heating 
<br>

## important note 

This project is still **in development**. <br>
Although everything works in general and I have already soldered several boards, changes in the code may be added or adapted due to improvements or bug fixes.

---

## PID Controller (main_improved.py)

The original firmware used a simple on/off approach with a fixed 25 % heater duty cycle. The improved firmware (`main_improved.py`) replaces this with a full **PID controller** and a **reflow soldering profile**.

### How it works

The heater is driven by a 1 Hz PWM signal on the SSR. Each control cycle the PID calculates the required duty cycle (0–100 %) based on the error between the current (filtered) temperature and the dynamic setpoint.

| Parameter | Value | Description |
|---|---|---|
| `kp` | 0.75 | Proportional gain |
| `ki` | 0.03 | Integral gain (anti-windup, clamped ±10) |
| `kd` | 0.45 | Derivative gain |
| `THERMAL_TAU_S` | 38 s | Thermal time constant of the iron |
| `THERMAL_DEAD_TIME_S` | 5 s | Transport delay from heater to sensor |

The dead time is used for **predictive temperature estimation**: the controller looks ahead by `THERMAL_DEAD_TIME_S` seconds so it starts reducing power before the setpoint is reached, minimising overshoot.

A spike-rejection filter (`MAX_TEMP_STEP_C_PER_S = 6.0 °C/s`) and an exponential smoothing filter (`TEMP_FILTER_ALPHA = 0.35`) clean up the MAX6675 readings before they enter the PID.

### Thermal parameter tuning

Run `measure_inertia.py` once to record three step-response tests (50 %, 75 %, 100 % duty) with fan-assisted cooling between runs. The script saves all data as JSON. Analyse the JSON to extract `tau` and `dead_time` and set the corresponding constants in `main_improved.py`.

### Reflow soldering profile

When a heating job is started the setpoint follows a 3-phase curve:

| Phase | Fraction of total time | Description |
|---|---|---|
| Preheat | 0 – 55 % | Ramp from ambient to ~55 % of peak temperature |
| Soak | 55 – 85 % | Hold at ~85 % of peak temperature |
| Reflow / Peak | 85 – 100 % | Ramp to peak temperature and hold |
| Cooldown | after peak hold | Fan on, green LEDs pulse, wait until < 80 °C |

### BLE command reference

| Command | Effect |
|---|---|
| `time,X,temp,Y` | Start reflow: duration X seconds, peak temperature Y °C |
| `A1` | Stop heating, fan on |
| `B1` | Toggle work light (Pin 17) |
| `K1` | Toggle fan |
| `K0` | Fan off |
| `KA` | Fan automatic (follows heater state) |
| `BR` | Break / emergency stop |

---

## CYD ESP32 Display Controller

In addition to the smartphone app, a **CYD (Cheap Yellow Display) ESP32** board can be used as a standalone controller and live visualiser for BLEIron.

The CYD connects to BLEIron via BLE (Nordic UART Service, same UUIDs as the app) and provides:

<p align="center">
  <img height="320" src="/CYD Display/Images/GUI.jpg" alt="CYD touch GUI">
</p>

- **Touch-based controls** — set target temperature and duration directly on the display
- **Live temperature chart** — a scrolling line chart shows the actual temperature vs. the reflow profile setpoint over time

<p align="center">
  <img height="320" src="/CYD Display/Images/Chart_fullscreen.jpg" alt="CYD fullscreen temperature chart">
</p>

- **Status display** — current phase (Preheat / Soak / Reflow / Cooldown), elapsed time, and heater duty cycle
- **Fan & light control** — dedicated buttons send `K1`, `K0`, `KA` and `B1` BLE commands

The CYD firmware is developed separately as an Arduino / ESP-IDF project. Because it uses the same BLE UART interface as the mobile app, both can coexist and the user can switch between them at any time.

---
