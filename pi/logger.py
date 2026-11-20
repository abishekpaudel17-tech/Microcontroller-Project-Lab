"""CSV waypoint log. Arrival does NOT imply a seed was dispensed."""
import csv
from pathlib import Path
import config as C

class WaypointLogger:
    def __init__(self,path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.file = self.path.open('w',newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['route','waypoint','column','row','target_x_cm','target_y_cm',
                              'fused_x_cm','fused_y_cm','encoder_x_cm','encoder_y_cm',
                              'pmw_x_cm','pmw_y_cm','heading_deg','estimated_error_cm','status'])

    def record(self,route,index,cell,pose,heading):
        err = pose.error(cell)
        self.writer.writerow([route,index,cell[0],cell[1],cell[0]*C.GRID_CM,cell[1]*C.GRID_CM,
                              round(pose.x,3),round(pose.y,3),round(pose.enc_x,3),round(pose.enc_y,3),
                              round(pose.pmw_x,3),round(pose.pmw_y,3),round(heading,3),round(err,3),'ARRIVED_NOT_PLANTED'])
        self.file.flush()
        print('WAYPOINT %02d %s | fused (%.2f, %.2f) | estimated error %.2f cm' %
              (index,cell,pose.x,pose.y,err))

    def close(self):
        self.file.close()
