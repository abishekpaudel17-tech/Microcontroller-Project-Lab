"""Conservative segment fusion, preserving raw PMW and encoder pose."""
import math
import config as C

def rotate(forward,left,heading):
    theta = math.radians(heading)
    return (forward*math.cos(theta)-left*math.sin(theta),
            forward*math.sin(theta)+left*math.cos(theta))

def reliability(enc_cm,pmw_forward,pmw_lateral,valid,total):
    ratio = pmw_forward/enc_cm if enc_cm > 0.01 else 0.0
    fraction = valid/total if total else 0.0
    good = (enc_cm >= C.FUSION_MIN_TRAVEL_CM
            and fraction >= C.FUSION_VALID_RATIO
            and C.FUSION_RATIO_MIN <= ratio <= C.FUSION_RATIO_MAX
            and abs(pmw_lateral) <= C.FUSION_MAX_LATERAL_CM)
    return good,ratio,fraction

class Pose:
    def __init__(self):
        self.x = self.y = 0.0
        self.enc_x = self.enc_y = 0.0
        self.pmw_x = self.pmw_y = 0.0

    def commit_forward(self,enc_cm,heading,pmw_dx,pmw_dy,valid,total):
        theta = math.radians(heading)
        along = pmw_dx*math.cos(theta)+pmw_dy*math.sin(theta)
        lateral = -pmw_dx*math.sin(theta)+pmw_dy*math.cos(theta)
        good,ratio,fraction = reliability(enc_cm,along,lateral,valid,total)
        weight = C.FUSION_PMW_WEIGHT if good else 0.0
        ex,ey = rotate(enc_cm,0.0,heading)
        self.enc_x += ex; self.enc_y += ey
        self.pmw_x += pmw_dx; self.pmw_y += pmw_dy
        self.x += (1-weight)*ex + weight*pmw_dx
        self.y += (1-weight)*ey + weight*pmw_dy
        return {'pmw_accepted':good,'pmw_weight':weight,'pmw_ratio':ratio,
                'pmw_valid_fraction':fraction,'pmw_lateral_cm':lateral}

    def commit_turn(self,dx,dy,valid,total):
        shift = math.hypot(dx,dy)
        good = total>0 and valid/total >= C.TURN_PMW_MIN_VALID_RATIO and shift <= C.TURN_PMW_MAX_SHIFT_CM
        if good:
            self.pmw_x += dx; self.pmw_y += dy
            self.x += C.TURN_PMW_FUSION_WEIGHT*dx
            self.y += C.TURN_PMW_FUSION_WEIGHT*dy
        return good

    def error(self,cell):
        return math.hypot(self.x-cell[0]*C.GRID_CM,self.y-cell[1]*C.GRID_CM)
