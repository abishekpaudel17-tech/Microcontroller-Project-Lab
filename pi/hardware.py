"""Pi 5 USB serial transport for Pico 2; simulator implements same interface."""
import math
import time
import config as C

NAMES = ('FL','LR','FR','RR')

class PicoMotors:
    def __init__(self,port=C.SERIAL_PORT):
        import serial
        self.serial = serial.Serial(port,C.BAUD,timeout=0.35,write_timeout=0.35)
        time.sleep(1.8)  # USB CDC reset on some firmware versions
        self.serial.reset_input_buffer()
        if self._ask('PING') != 'PONG':
            raise RuntimeError('Pico handshake failed')
        self.stop()

    def _ask(self,command):
        self.serial.write((command+'\n').encode())
        answer = self.serial.readline().decode(errors='replace').strip()
        if not answer or answer.startswith('ERR'):
            raise RuntimeError('Pico response for %s: %r' % (command,answer))
        return answer

    def drive(self,values):
        cmd = 'DRIVE ' + ' '.join('%.4f'%values[n] for n in NAMES)
        if self._ask(cmd) != 'OK': raise RuntimeError('Pico DRIVE not acknowledged')

    def counts(self):
        reply = self._ask('STATUS').split()
        if len(reply)!=5 or reply[0]!='COUNTS':
            raise RuntimeError('Bad Pico encoder reply: %s'%reply)
        return dict(zip(NAMES,(int(v) for v in reply[1:])))

    def stop(self):
        if self._ask('STOP')!='OK': raise RuntimeError('Pico STOP failed')

    def close(self):
        try: self.stop()
        finally: self.serial.close()

class SimHardware:
    """Deterministic kinematic simulator, NOT a physics or skid model."""
    def __init__(self):
        self.values = {n:0.0 for n in NAMES}
        self.counts_float = {n:0.0 for n in NAMES}
        self.heading = 0.0
        self.x = self.y = 0.0
        self.pmw_x = self.pmw_y = 0.0
        self.last = time.monotonic()

    def _advance(self):
        now = time.monotonic()
        dt = min(0.1,now-self.last)
        self.last = now
        l = (self.values['FL']+self.values['LR'])/2
        r = (self.values['FR']+self.values['RR'])/2
        speed = (l+r)/2*C.SIM_CM_PER_S_AT_FULL_PWM
        omega = (r-l)*C.SIM_TURN_DEG_PER_S_AT_FULL_DIFFERENTIAL
        mid = math.radians(self.heading+omega*dt/2)
        dx = speed*dt*math.cos(mid); dy = speed*dt*math.sin(mid)
        self.x += dx; self.y += dy
        self.pmw_x += dx; self.pmw_y += dy
        self.heading += omega*dt
        for n in NAMES:
            distance = abs(self.values[n])*C.SIM_CM_PER_S_AT_FULL_PWM*dt
            self.counts_float[n] += distance*C.COUNTS_PER_REV/(math.pi*C.WHEEL_DIAMETER_CM)

    def drive(self,values):
        self._advance()
        self.values = dict(values)

    def counts(self):
        self._advance()
        return {n:int(self.counts_float[n]) for n in NAMES}

    def stop(self):
        self._advance()
        self.values = {n:0.0 for n in NAMES}

    def read_flow(self):
        self._advance()
        # World delta converted to body coordinates by navigation's sensor wrapper
        return self.pmw_x,self.pmw_y

    def close(self): self.stop()
