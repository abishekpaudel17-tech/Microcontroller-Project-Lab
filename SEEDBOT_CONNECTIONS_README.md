# Seed Bot — Raspberry Pi 5, Pico 2, and L298N Connections

This wiring guide matches the proposed pin assignments in the **`seedbot_navigation_pi5_pico2_L298N.zip`** code. It is a **proposed wiring map**, not a record of hardware that has been verified on your assembled rover.

## 1. System architecture

```text
Battery / motor supply ───────────┬── L298N LEFT  ── Front-left + Rear-left motors
                                  └── L298N RIGHT ── Front-right + Rear-right motors
                                        ▲
                                        │ PWM + direction (3.3 V GPIO signals)
Raspberry Pi 5 ── USB data/power ── Pico 2
      │                                 ▲
      ├── I2C ── MPU6050                └── Four encoder A signals (level-shifted)
      └── SPI ── PMW3901
```

**Pi 5:** runs route planning, MPU6050/PMW3901 sensing, sensor fusion, navigation, and waypoint CSV logging. **Pico 2:** drives the L298N inputs, counts encoder pulses, and communicates with Pi over USB serial.

## 2. Raspberry Pi 5 connections

### MPU6050 (I²C)

| MPU6050 | Pi 5 | Physical pin |
|---|---|---:|
| VCC | 3.3 V **only if your breakout supports it** | 1 |
| GND | GND | 6 |
| SDA | GPIO2 / SDA1 | 3 |
| SCL | GPIO3 / SCL1 | 5 |

The IMU code expects I²C address `0x68`. Verify your MPU6050 breakout's supply voltage and logic-level requirements before powering it. Do not feed 5 V I²C pull-ups into Pi GPIO.

### PMW3901 (SPI)

| PMW3901 breakout | Pi 5 | Physical pin |
|---|---|---:|
| 3V3 / VCC | 3.3 V **if supported by the breakout** | 17 |
| GND | GND | 20 |
| SCK / CLK | GPIO11 / SPI0 SCLK | 23 |
| MOSI | GPIO10 / SPI0 MOSI | 19 |
| MISO | GPIO9 / SPI0 MISO | 21 |
| CS | GPIO8 / SPI0 CE0 | 24 |

The repository's Pi code uses the Pimoroni PMW3901 Python library. Check your exact breakout labels and voltage requirements. Enable I²C and SPI on Pi using `sudo raspi-config`.

### Pi 5 ↔ Pico 2

Connect the **Pico 2 USB connector to a Pi 5 USB-A port** using a suitable USB data cable. This supplies Pico power and provides USB serial communication (often `/dev/ttyACM0`; check on your Pi). No extra Pi GPIO UART wires are needed for this design.

## 3. Pico 2 → two L298N boards

Use **two L298N modules**, each with two H-bridge channels. Remove the **ENA and ENB jumpers** on both modules to allow PWM speed control. The code uses **GPIO numbers (GP)**, not physical Pico header pin numbers.

| Motor | Driver / channel | Pico PWM → EN | Pico IN1 | Pico IN2 | Pico encoder A |
|---|---|---|---|---|---|
| Front-left (FL) | LEFT / A | GP13 (pin 17) → ENA | GP14 (pin 19) → IN1 | GP15 (pin 20) → IN2 | GP2 (pin 4) |
| Rear-left (LR) | LEFT / B | GP10 (pin 14) → ENB | GP11 (pin 15) → IN3 | GP12 (pin 16) → IN4 | GP4 (pin 6) |
| Front-right (FR) | RIGHT / A | GP16 (pin 21) → ENA | GP17 (pin 22) → IN1 | GP18 (pin 24) → IN2 | GP6 (pin 9) |
| Rear-right (RR) | RIGHT / B | GP19 (pin 25) → ENB | GP20 (pin 26) → IN3 | GP21 (pin 27) → IN4 | GP8 (pin 11) |

Connect each motor's **two power leads** to its corresponding L298N motor output pair (`OUT1/OUT2` for channel A; `OUT3/OUT4` for channel B). The original code's direction signs are FL −1, LR +1, FR +1, RR −1. **Confirm each wheel's direction with the rover lifted before driving on the floor.**

**Pico ground:** connect Pico GND (for example physical pin 18) to the GND terminals on **both** L298N boards and the encoder power-supply ground. All control signals require a common ground.

**GP22 is unused** in this L298N version: the previous TB6612 standby (`STBY`) line does not exist on L298N.

## 4. Encoder connections

For each FIT0450 encoder, connect its supply and GND according to its specific encoder specifications, and send **channel A** to the Pico pin shown above. The code counts rising edges on channel A only; channel B is not used in this version.

**Critical:** Pico 2 GPIO inputs are **not 5 V tolerant**. If encoder A can rise to 5 V, install a verified logic-level shifter or voltage divider **on every encoder signal** before it reaches the Pico. For example, a divider using **10 kΩ from encoder A to Pico GPIO** and **20 kΩ from Pico GPIO to GND** yields approximately 3.33 V for a 5 V input. Measure it with a multimeter before connecting the Pico. Check the signal at encoder pulse frequency, not only when stationary. Never power a 5 V encoder from Pico 3V3 without confirming the encoder supports 3.3 V operation.

## 5. Motor and logic power

- Feed the appropriate motor supply to the **motor supply input** of both L298N modules; connect its return to their GND terminals. **Do not connect motor supply voltage to Pi 5 or Pico 2 GPIO.**
- Power Pi 5 from a **separate, adequately rated regulated USB-C supply**. Do not use the L298N onboard 5 V output as the Pi power source.
- Power Pico 2 via USB from Pi 5. Do not also feed an unverified L298N 5 V output into Pico 3V3.
- The L298N's **5V-EN regulator jumper** and 5 V logic terminal behavior depend on the board and motor-supply voltage. Follow the exact module's documentation; **do not confuse the 5V-EN jumper with the ENA/ENB speed-control jumpers**.
- L298N drivers lose appreciable voltage and can overheat. Verify FIT0450 motor **running, startup, and stall currents** against the board's real current/thermal capability before full-load testing.
- Use a physical motor power disconnect / emergency stop. A USB communication watchdog is not a substitute for a hardware cutoff.

## 6. Repository files

```text
seedbot-navigation/
├── pi/
│   ├── main.py          # route selection / simulation / real run
│   ├── routes.py        # four 4×4 routes, 25 cm spacing
│   ├── navigation.py    # movement, heading control, turns
│   ├── hardware.py      # USB communication with Pico
│   ├── imu.py           # MPU6050
│   ├── pmw3901.py       # PMW3901 Pi adapter
│   ├── fusion.py        # position fusion
│   └── logger.py        # waypoint CSV output
├── pico/
│   ├── main.py          # USB command listener
│   ├── config.py        # GPIO mapping
│   ├── motor_control.py # L298N PWM / direction
│   └── encoders.py      # encoder A pulse counts
├── docs/wiring.md
└── README.md
```

## 7. Setup and safe first tests

1. **Without motor power**, check every connection, polarity, common ground, encoder voltage divider, and L298N jumper.
2. Install MicroPython on Pico 2 and copy the four `pico/` files to the Pico filesystem root.
3. On Pi 5, enable I²C/SPI and install dependencies following the repository's main README. Confirm the IMU at `0x68`, PMW identification, and Pico USB serial device.
4. Test routes in simulation first:

   ```bash
   cd pi
   python3 main.py --simulate --route 4
   ```

5. With **wheels lifted**, apply motor power and check one wheel at a time: forward sign, encoder pulses, and watchdog stop behavior.
6. Recheck gyro yaw sign, optical-flow axes, wheel diameter, encoder counts/revolution, and PMW scale on a measured floor distance.
7. Only after those checks, run a short floor test. The repository's real-hardware command is:

   ```bash
   cd pi
   python3 main.py --route 4 --port /dev/ttyACM0 --confirm-hardware
   ```

   Replace `/dev/ttyACM0` with the actual device path.

## 8. Important limitations

This is a **proposed integration wiring guide** matching the generated repository, **not a verified electrical schematic**. The Pi 5 navigation port has not been validated on the actual rover. The waypoint errors are estimates from sensors, not independent physical position measurements. Waypoint arrival logs do **not** confirm that a seed was dispensed. The PMW Pi adapter currently has more limited quality validation than your original MicroPython PMW driver; recalibrate and test before trusting position corrections.
