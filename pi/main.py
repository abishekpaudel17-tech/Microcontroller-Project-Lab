"""Run from pi/ directory: python3 main.py --simulate --route 4"""
import argparse
import sys
from datetime import datetime
from pathlib import Path
from hardware import PicoMotors, SimHardware
from navigation import Navigator, Sensors
from routes import ROUTES

def main():
    parser=argparse.ArgumentParser(description='Seed Bot Pi 5 navigation')
    parser.add_argument('--route',type=int,choices=sorted(ROUTES),default=4)
    parser.add_argument('--simulate',action='store_true',help='No GPIO, sensors or motors')
    parser.add_argument('--port',default=None,help='Pico USB serial port')
    parser.add_argument('--log',default=None,help='Waypoint CSV output')
    parser.add_argument('--confirm-hardware',action='store_true',help='Required to energize real motors')
    args=parser.parse_args()
    if not args.simulate and not args.confirm_hardware:
        parser.error('Real hardware requires --confirm-hardware; test --simulate first')
    motors=None; imu=None; nav=None
    try:
        if args.simulate:
            import config as C
            C.PAUSE_AFTER_WAYPOINT_S = 0.0
            C.SETTLE_S = 0.02
            motors=SimHardware()
            sensors=Sensors(sim=motors)
        else:
            from imu import MPU6050
            from pmw3901 import PMW3901
            import config as C
            motors=PicoMotors(args.port or C.SERIAL_PORT)
            imu=MPU6050()
            imu.calibrate()
            pmw=PMW3901()
            sensors=Sensors(imu=imu,pmw=pmw)
        log=args.log or str(Path(__file__).resolve().parents[1]/'logs'/
                            ('route%d_%s.csv'%(args.route,datetime.now().strftime('%Y%m%d_%H%M%S'))))
        nav=Navigator(motors,sensors,args.route,log)
        print('Starting route %d | simulation=%s | log=%s'%(args.route,args.simulate,log))
        nav.run()
    except KeyboardInterrupt:
        print('Emergency stop requested',file=sys.stderr)
        raise SystemExit(130)
    finally:
        if nav: nav.close()
        if motors: motors.close()
        if imu: imu.close()

if __name__=='__main__':main()
