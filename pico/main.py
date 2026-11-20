"""Pico 2 MicroPython USB serial motor service. Power up with wheels off floor.
Protocol: PING, STOP, STATUS, DRIVE fl lr fr rr (each -1..1).
Response: PONG, OK, COUNTS n n n n, ERR message.
Safety: automatic stop when no DRIVE command for WATCHDOG_MS.
"""
import sys
import time
import uselect
from config import MOTOR_CONFIG, WATCHDOG_MS
from motor_control import Motor
from encoders import Encoder, snapshot

motors = {}
encoders = {}

def stop_all():
    for m in motors.values():
        m.stop()

def run():
    for name,cfg in MOTOR_CONFIG.items():
        motors[name] = Motor(name,cfg)
        encoders[name] = Encoder(cfg[3])
    stop_all()
    poll = uselect.poll()
    poll.register(sys.stdin,uselect.POLLIN)
    last_drive = time.ticks_ms()
    armed = False
    while True:
        if armed and time.ticks_diff(time.ticks_ms(),last_drive)>WATCHDOG_MS:
            stop_all()
            armed = False
        if not poll.poll(20):
            continue
        line = sys.stdin.readline().strip()
        parts = line.split()
        if not parts: continue
        cmd = parts[0].upper()
        try:
            if cmd == 'PING':
                print('PONG')
            elif cmd == 'STOP':
                stop_all(); armed=False; print('OK')
            elif cmd == 'STATUS':
                print('COUNTS ' + ' '.join(str(v) for v in snapshot(encoders)))
            elif cmd == 'DRIVE' and len(parts)==5:
                values = [float(v) for v in parts[1:]]
                if any(not (-1 <= v <= 1) for v in values):
                    raise ValueError('PWM outside [-1,1]')
                for name,value in zip(('FL','LR','FR','RR'),values):
                    motors[name].drive(value)
                last_drive = time.ticks_ms()
                armed = True
                print('OK')
            else:
                print('ERR unknown command')
        except Exception as exc:
            stop_all(); armed=False
            print('ERR ' + str(exc))

try:
    run()
except BaseException:
    stop_all()
    raise
