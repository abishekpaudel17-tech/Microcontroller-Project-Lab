"""Seed Bot route executor. Port of the supplied Pico controller's control ideas.

Pi controls navigation; Pico 2 drives four motors and counts encoder A edges.
Coordinates and pose are estimates, NOT independently measured ground truth.
"""
import math
import time
import config as C
from routes import ROUTES, heading_for_move, signed_delta
from fusion import Pose, rotate, reliability
from logger import WaypointLogger

NAMES = ('FL','LR','FR','RR')

def clamp(x,a,b): return max(a,min(b,x))

def count_distance(n): return n*math.pi*C.WHEEL_DIAMETER_CM/C.COUNTS_PER_REV

def mean_distance(current,start):
    return count_distance(sum(current[n]-start[n] for n in NAMES)/4.0)

class Sensors:
    """Samples heading and optical flow; returns WORLD-coordinate PMW displacement."""
    def __init__(self,imu=None,pmw=None,sim=None):
        self.imu,self.pmw,self.sim = imu,pmw,sim
        self.heading = 0.0
        self.last_heading = 0.0
        self.sim_last_x = self.sim_last_y = 0.0

    def sample(self):
        if self.sim is not None:
            self.sim._advance()
            self.heading = self.sim.heading
            x,y = self.sim.read_flow()
            dx,dy = x-self.sim_last_x,y-self.sim_last_y
            self.sim_last_x,self.sim_last_y = x,y
            self.last_heading = self.heading
            return dx,dy,True
        self.imu.update()
        self.heading = self.imu.heading
        dx,dy,quality,valid = self.pmw.read_motion()
        forward = dy*C.PMW_CM_PER_COUNT*C.PMW_Y_SIGN
        left = dx*C.PMW_CM_PER_COUNT*C.PMW_X_SIGN
        mid_heading = (self.last_heading+self.heading)/2
        wx,wy = rotate(forward,left,mid_heading)
        self.last_heading = self.heading
        return wx,wy,valid

class Navigator:
    def __init__(self,motors,sensors,route,log_path):
        self.motors = motors
        self.sensors = sensors
        self.pose = Pose()
        self.route = route
        self.log = WaypointLogger(log_path)
        self.heading_target = 0.0
        self.last_status = None

    def _sample(self,acc):
        dx,dy,valid = self.sensors.sample()
        acc['total'] += 1
        if valid:
            acc['valid'] += 1
            acc['dx'] += dx; acc['dy'] += dy
        return dx,dy

    def _settle(self,acc,seconds=C.SETTLE_S):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            self._sample(acc)
            time.sleep(C.LOOP_S)

    def _stop(self):
        self.motors.stop()

    def _forward(self,target_cell):
        theta = math.radians(self.heading_target)
        tx,ty = target_cell[0]*C.GRID_CM,target_cell[1]*C.GRID_CM
        along = (tx-self.pose.x)*math.cos(theta)+(ty-self.pose.y)*math.sin(theta)
        if not 19.0<=along<=30.0:
            raise RuntimeError('Unsafe forward projection %.2f cm'%along)
        print('FORWARD %.2f cm to %s'%(along,target_cell))
        start = self.motors.counts()
        prev = dict(start)
        now = time.monotonic()
        wheel_last = {n:now for n in NAMES}
        last_progress = now
        last_distance = 0.0
        started = now
        acc = {'dx':0.0,'dy':0.0,'valid':0,'total':0}
        initial_heading = self.sensors.heading
        stop_threshold = max(0.2,along-C.COAST_COMP_CM)
        try:
            while True:
                self._sample(acc)
                counts = self.motors.counts()
                dist = mean_distance(counts,start)
                pmw_along = acc['dx']*math.cos(theta)+acc['dy']*math.sin(theta)
                pmw_lateral = -acc['dx']*math.sin(theta)+acc['dy']*math.cos(theta)
                good,_,_ = reliability(dist,pmw_along,pmw_lateral,acc['valid'],acc['total'])
                weight = C.FUSION_PMW_WEIGHT if good else 0.0
                fused_dist = (1-weight)*dist+weight*pmw_along
                remaining = stop_threshold-fused_dist
                if remaining<=0: break
                now = time.monotonic()
                if now-started>C.FORWARD_TIMEOUT_S:
                    raise RuntimeError('Forward timeout')
                for n in NAMES:
                    if counts[n]>prev[n]: wheel_last[n]=now
                    elif now-wheel_last[n]>C.STALL_TIMEOUT_S:
                        raise RuntimeError('Encoder %s stalled'%n)
                prev = counts
                if dist>last_distance+0.05:
                    last_distance=dist;last_progress=now
                if now-last_progress>C.STALL_TIMEOUT_S:
                    raise RuntimeError('Encoder distance stalled')
                if remaining>C.SLOW_START_CM: base=C.BASE_PWM
                else:
                    fraction=clamp(remaining/C.SLOW_START_CM,0,1)
                    base=C.SLOW_PWM+(C.BASE_PWM-C.SLOW_PWM)*fraction
                heading_cmd=self.heading_target
                if good and remaining> C.CROSS_TRACK_MIN_REMAINING_CM:
                    live_x=self.pose.x+(1-weight)*dist*math.cos(theta)+weight*acc['dx']
                    live_y=self.pose.y+(1-weight)*dist*math.sin(theta)+weight*acc['dy']
                    cross=-(tx-live_x)*math.sin(theta)+(ty-live_y)*math.cos(theta)
                    offset=math.degrees(math.atan2(C.CROSS_TRACK_KP*cross,max(remaining,5)))
                    heading_cmd+=clamp(offset,-C.CROSS_TRACK_MAX_HEADING_DEG,C.CROSS_TRACK_MAX_HEADING_DEG)
                error=heading_cmd-self.sensors.heading
                correction=clamp(C.STEERING_SIGN*C.KP_HEADING*error,-C.MAX_CORRECTION,C.MAX_CORRECTION)
                left=clamp(base-correction,C.MIN_FORWARD_PWM,C.MAX_FORWARD_PWM)
                right=clamp(base+correction,C.MIN_FORWARD_PWM,C.MAX_FORWARD_PWM)
                self.motors.drive({'FL':left,'LR':left,'FR':right,'RR':right})
                time.sleep(C.LOOP_S)
        finally:
            self._stop()
        self._settle(acc)
        settled=mean_distance(self.motors.counts(),start)
        mean_heading=(initial_heading+self.sensors.heading)/2
        report=self.pose.commit_forward(settled,mean_heading,acc['dx'],acc['dy'],acc['valid'],acc['total'])
        print('Distance %.2f cm | PMW accepted %s | fused XY %.2f,%.2f'%
              (settled,report['pmw_accepted'],self.pose.x,self.pose.y))

    def _turn(self,target):
        print('TURN to %+.1f deg'%target)
        direction=1 if target>self.sensors.heading else -1
        start_heading=self.sensors.heading
        start=self.motors.counts()
        previous=dict(start)
        pwm_adjust={n:0.0 for n in NAMES}
        pwm={n:C.TURN_PWM for n in NAMES}
        acc={'dx':0.0,'dy':0.0,'valid':0,'total':0}
        started=time.monotonic(); last_control=started;last_motion=started
        try:
            while True:
                self._sample(acc)
                now=time.monotonic()
                error=target-self.sensors.heading
                remaining=abs(error)
                if remaining<=C.TURN_TOLERANCE_DEG: break
                if direction*error<0: break
                if now-started>C.TURN_TIMEOUT_S: raise RuntimeError('Turn timeout')
                progress=direction*(self.sensors.heading-start_heading)
                if progress < -3: raise RuntimeError('Turn in wrong direction')
                if now-started>1.8 and progress<3: raise RuntimeError('Turn not progressing')
                if remaining>C.TURN_SLOW_AT_DEG: base=C.TURN_PWM
                elif remaining>C.TURN_FINAL_AT_DEG: base=C.TURN_SLOW_PWM
                else: base=C.TURN_FINAL_PWM
                if now-last_control>=0.1:
                    counts=self.motors.counts()
                    inc={n:counts[n]-previous[n] for n in NAMES}
                    totals={n:counts[n]-start[n] for n in NAMES}
                    avg_inc=sum(inc.values())/4
                    avg_total=sum(totals.values())/4
                    for n in NAMES:
                        speed_term=C.TURN_SPEED_KP*(avg_inc-inc[n])/max(avg_inc,1)
                        distance_term=C.TURN_DISTANCE_KP*(avg_total-totals[n])/max(avg_total,1)
                        adjust=clamp(speed_term+distance_term,-C.TURN_MAX_ADJUST,C.TURN_MAX_ADJUST)
                        pwm_adjust[n]=0.65*pwm_adjust[n]+0.35*adjust
                        pwm[n]=clamp(base+pwm_adjust[n],C.TURN_MIN_PWM,C.TURN_MAX_PWM)
                    if sum(inc.values())>0:last_motion=now
                    previous=counts;last_control=now
                if now-last_motion>C.STALL_TIMEOUT_S:raise RuntimeError('Turn encoder stall')
                self.motors.drive({'FL':-direction*pwm['FL'],'LR':-direction*pwm['LR'],
                                   'FR':direction*pwm['FR'],'RR':direction*pwm['RR']})
                time.sleep(C.LOOP_S)
        finally:
            self._stop()
        self._settle(acc)
        if abs(target-self.sensors.heading)>C.TURN_TOLERANCE_DEG:
            raise RuntimeError('Turn settled outside tolerance: %.2f deg'%self.sensors.heading)
        accepted=self.pose.commit_turn(acc['dx'],acc['dy'],acc['valid'],acc['total'])
        print('Turn heading %.2f | PMW shift accepted %s'%(self.sensors.heading,accepted))

    def run(self):
        cells=ROUTES[self.route]
        self._sample({'dx':0.0,'dy':0.0,'valid':0,'total':0})
        self.log.record(self.route,1,cells[0],self.pose,self.sensors.heading)
        for i,(a,b) in enumerate(zip(cells,cells[1:]),start=2):
            desired=heading_for_move(a,b)
            delta=signed_delta(desired,self.heading_target)
            if delta==-180:delta=180
            if abs(delta)==180:
                step=90 if delta>0 else -90
                for _ in range(2):
                    self._turn(self.heading_target+step)
                    self.heading_target+=step
                    self._settle({'dx':0.0,'dy':0.0,'valid':0,'total':0},C.PAUSE_AFTER_WAYPOINT_S)
            elif delta:
                self._turn(self.heading_target+delta)
                self.heading_target+=delta
                self._settle({'dx':0.0,'dy':0.0,'valid':0,'total':0},C.PAUSE_AFTER_WAYPOINT_S)
            self._forward(b)
            self._settle({'dx':0.0,'dy':0.0,'valid':0,'total':0},C.PAUSE_AFTER_WAYPOINT_S)
            self.log.record(self.route,i,b,self.pose,self.sensors.heading)
        print('ROUTE %d COMPLETE: 16 waypoints; final estimated error %.2f cm'%
              (self.route,self.pose.error(cells[-1])))

    def close(self):
        try: self._stop()
        finally: self.log.close()
