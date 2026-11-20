"""Rising-edge-only encoder A counter (unsigned magnitude)."""
from machine import Pin, disable_irq, enable_irq

class Encoder:
    def __init__(self,pin):
        self.count = 0
        self.pin = Pin(pin,Pin.IN,Pin.PULL_UP)
        self.pin.irq(trigger=Pin.IRQ_RISING,handler=self._tick)

    def _tick(self,pin):
        self.count += 1

def snapshot(encoders):
    state = disable_irq()
    try:
        return [encoders[n].count for n in ('FL','LR','FR','RR')]
    finally:
        enable_irq(state)
