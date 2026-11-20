"""Pico 2 GPIO assignments for TWO L298N boards; see docs/wiring.md.
Each tuple: (ENA_PWM, IN1, IN2, encoder_A, forward_sign).
Remove ENA/ENB jumpers; never apply 5V to Pico GPIO.
"""
MOTOR_CONFIG = {
    'FL': (13,14,15,2,-1),
    'LR': (10,11,12,4,1),
    'FR': (16,17,18,6,1),
    'RR': (19,20,21,8,-1),
}
COUNTS_PER_REV = 971.0
WATCHDOG_MS = 600
PWM_HZ = 1000
