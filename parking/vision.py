"""Image-only parking perception for approximately top-down RGB cameras.

Requires opencv-python; object inference additionally requires ultralytics and
an explicitly supplied, locally available YOLO detection checkpoint.
"""
import cv2
import numpy as np


class ParkingDetector:
    def __init__(self, weights=None):
        self.model = None
        if weights:
            from pathlib import Path
            if not Path(weights).is_file():
                raise ValueError('Parking weights must be an existing local checkpoint')
            from ultralytics import YOLO
            self.model = YOLO(str(Path(weights).expanduser()))
        self.slots = []
        self.summary = {}

    def process(self, rgb):
        """Return annotated RGB image and slot records; no simulator actor data used.

        Free means no detected object overlaps the slot, not certified clearance.
        Without an object model all slots remain unknown.
        """
        h, w = rgb.shape[:2]
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        white = cv2.inRange(hsv, (0, 0, 175), (179, 65, 255))
        lines = cv2.HoughLinesP(white, 1, np.pi / 180, 25,
                                minLineLength=max(20, h // 25), maxLineGap=8)
        dividers = []
        if lines is not None:
            for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
                if abs(x2-x1) < max(5, abs(y2-y1)*0.12):
                    dividers.append(((x1+x2)//2, min(y1,y2), max(y1,y2)))
        # Merge the two edges and fragmented detections of each painted divider.
        merged = []
        for x, top, bottom in sorted(dividers):
            for i, (mx, mt, mb) in enumerate(merged):
                if abs(x-mx) < max(5, w*.012) and min(bottom,mb)-max(top,mt) > 0:
                    merged[i] = ((x+mx)//2, min(top,mt), max(bottom,mb))
                    break
            else:
                merged.append((x,top,bottom))
        # Consolidate again: incremental averaging can leave duplicate fragments.
        consolidated = []
        for x, top, bottom in sorted(merged):
            if consolidated and abs(x-consolidated[-1][0]) < max(5, w*.012) and min(bottom, consolidated[-1][2]) > max(top, consolidated[-1][1]):
                px, pt, pb = consolidated.pop()
                consolidated.append(((int(x)+int(px))//2, min(top,pt), max(bottom,pb)))
            else:
                consolidated.append((x,top,bottom))
        merged = consolidated
        objects = []
        if self.model is not None:
            result = self.model.predict(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                        conf=0.25, verbose=False)[0]
            for box in result.boxes:
                objects.append(tuple(map(int, box.xyxy[0].cpu().tolist())))
        candidates = []
        for x, top, bottom in merged:
            neighbors = [(nx,nt,nb) for nx,nt,nb in merged
                         if nx > x and min(bottom,nb)-max(top,nt) > .65*min(bottom-top,nb-nt)]
            if not neighbors:
                continue
            nx, nt, nb = min(neighbors)
            y1, y2 = max(top,nt), min(bottom,nb)
            width, depth = nx-x, y2-y1
            if not (.025*w < width < .12*w and 1.1 < depth/width < 3.5):
                continue
            rect = (x,y1,nx,y2)
            if rect in candidates:
                continue
            candidates.append(rect)
        output = rgb.copy()
        self.slots = []
        for index, (x1,y1,x2,y2) in enumerate(sorted(candidates, key=lambda r:(r[1],r[0]))):
            area = (x2-x1)*(y2-y1)
            overlap = max([max(0,min(x2,b[2])-max(x1,b[0])) *
                           max(0,min(y2,b[3])-max(y1,b[1])) / area for b in objects] or [0])
            status = 'unknown' if self.model is None else ('occupied' if overlap > .12 else 'free')
            color = {'unknown':(255,190,0), 'occupied':(255,60,60), 'free':(40,220,80)}[status]
            self.slots.append(dict(id=index, polygon=[[x1,y1],[x2,y1],[x2,y2],[x1,y2]],
                                   status=status, overlap=overlap))
            cv2.rectangle(output,(x1,y1),(x2,y2),color,2)
            cv2.putText(output, '%s %s' % (index, 'AVAILABLE' if status == 'free' else status.upper()),(x1+2,y1+14),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
        for x1,y1,x2,y2 in objects:
            cv2.rectangle(output,(x1,y1),(x2,y2),(80,160,255),1)
        self.summary = {state: [slot['id'] for slot in self.slots if slot['status'] == state]
                        for state in ('free', 'occupied', 'unknown')}
        # A dark banner keeps the current occupancy readable against pavement.
        cv2.rectangle(output, (0, 0), (w, 64), (20, 20, 20), -1)
        title = 'Available: %d   Occupied: %d   Unknown: %d' % tuple(
            len(self.summary[state]) for state in ('free', 'occupied', 'unknown'))
        cv2.putText(output, title, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (255, 255, 255), 1)
        detail = ('Available IDs: ' + ', '.join(map(str, self.summary['free']))) if self.model is not None else 'UNKNOWN: supply --parking-weights for object detection'
        if not self.slots:
            detail = 'No slots detected: check camera position, height and line alignment'
        cv2.putText(output, detail, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, .4,
                    (255, 210, 80), 1)
        return output, self.slots
