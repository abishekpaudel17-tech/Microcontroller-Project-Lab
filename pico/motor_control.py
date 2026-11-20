"""L298N direction and PWM. No TB6612 standby signal is used."""
from machine import Pin, PWM
from config import MOTOR_CONFIG, PWM_HZ

class Motor:
    def __init__(self,name,cfg):
        pwm_pin,in1,in2,enc_pin,sign = cfg
        self.name = name
        self.sign = sign
        self.in1 = Pin(in1,Pin.OUT,value=0)
        self.in2 = Pin(in2,Pin.OUT,value=0)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(PWM_HZ)
        self.pwm.duty_u16(0)

    def drive(self,duty):
        duty = max(-1.0,min(1.0,duty))
        if abs(duty)<0.001:
            self.stop(); return
        direction = self.sign if duty>0 else -self.sign
        self.in1.value(1 if direction>0 else 0)
        self.in2.value(0 if direction>0 else 1)
        self.pwm.duty_u16(int(abs(duty)*65535))

    def stop(self):
        self.pwm.duty_u16(0)
        self.in1.value(0); self.in2.value(0)
