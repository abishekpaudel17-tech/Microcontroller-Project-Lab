"""Pi 5 PMW3901 wrapper for Pimoroni pmw3901-python, NOT MicroPython SoftSPI.

Pimoroni get_motion() exposes dx/dy but not full burst quality. This adapter
marks successful samples as valid; calibrate and verify quality on hardware.
"""
class PMW3901:
    def __init__(self, spi_port=0, spi_cs=0):
        from pmw3901 import PMW3901 as PimoroniPMW3901
        self.sensor = PimoroniPMW3901(spi_port=spi_port, spi_cs=spi_cs)
        self.sensor.set_rotation(0)

    def read_motion(self):
        dx,dy = self.sensor.get_motion()
        return dx,dy,None,True
