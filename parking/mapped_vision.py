"""Mapped slot occupancy using world-to-image projection and cropped YOLO inference."""
import json
import math
from pathlib import Path
import cv2
import numpy as np
from .vision import ParkingDetector


def slot_corners(slot):
    angle = math.radians(slot['yaw_deg'])
    cosine, sine = math.cos(angle), math.sin(angle)
    corners = []
    for forward, lateral in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        dx = forward * slot['depth'] / 2
        dy = lateral * slot['width'] / 2
        corners.append((slot['x'] + dx * cosine - dy * sine,
                        slot['y'] + dx * sine + dy * cosine))
    return corners


class MappedParkingDetector(ParkingDetector):
    def __init__(self, weights, slot_file, ground_z=0, inference="crop", slice_size=512, slice_overlap=.25, confidence=.15):
        super().__init__(weights)
        self.layout = json.loads(Path(slot_file).read_text(encoding='utf-8'))['parking_slots']
        self.ground_z = ground_z
        self.history = {}
        self.confirmation_frames = 3
        self.inference = inference
        self.slice_size = slice_size
        self.slice_overlap = slice_overlap
        self.confidence = confidence
        if inference == 'sahi':
            from sahi import AutoDetectionModel
            self.sahi_model = AutoDetectionModel.from_pretrained(
                model_type='ultralytics', model=self.model,
                confidence_threshold=confidence, device=str(self.model.device))

    def process_frame(self, rgb, frame, fov, *, slots=None):
        h, w = rgb.shape[:2]
        focal = w / (2 * math.tan(math.radians(fov)/2))
        inverse = np.asarray(frame.transform.get_inverse_matrix())
        def project(x,y):
            local = inverse @ np.array([x,y,self.ground_z,1])
            if local[0] <= .01:
                return None
            return [focal*local[1]/local[0]+w/2, -focal*local[2]/local[0]+h/2]
        layout = self.layout if slots is None else slots
        projected = [(slot, [project(x, y) for x, y in slot_corners(slot)]) for slot in layout]
        points_in_front = [p for _, corners in projected for p in corners if p is not None]
        self.monitor_roi = [0, 0, w, h]
        if points_in_front:
            bounds = np.asarray(points_in_front)
            low = np.floor(bounds.min(axis=0)).astype(int) - 12
            high = np.ceil(bounds.max(axis=0)).astype(int) + 12
            left, top = max(0, int(low[0])), max(0, int(low[1]))
            right, bottom = min(w, int(high[0])), min(h, int(high[1]))
            if right > left and bottom > top:
                self.monitor_roi = [left, top, right, bottom]
        left, top, right, bottom = self.monitor_roi
        detections = []
        if self.inference == 'sahi' and points_in_front:
            from sahi.predict import get_sliced_prediction
            prediction = get_sliced_prediction(
                np.ascontiguousarray(rgb[top:bottom, left:right]), self.sahi_model,
                slice_height=self.slice_size, slice_width=self.slice_size,
                overlap_height_ratio=self.slice_overlap, overlap_width_ratio=self.slice_overlap,
                perform_standard_pred=False, verbose=0)
            for detection in prediction.object_prediction_list:
                x1,y1,x2,y2 = detection.bbox.to_xyxy()
                detections.append([x1+left,y1+top,x2+left,y2+top])
        output = rgb.copy()
        self.slots = []
        for slot, corners in projected:
            visible = all(p is not None and 0 <= p[0] < w and 0 <= p[1] < h for p in corners)
            status = 'unknown'
            overlap = 0.0
            if visible:
                polygon = np.asarray(corners,np.float32)
                area = cv2.contourArea(polygon)
                boxes = detections
                if self.inference == 'crop':
                    x1,y1 = np.floor(polygon.min(axis=0)).astype(int)
                    x2,y2 = np.ceil(polygon.max(axis=0)).astype(int)
                    left,top = max(0,x1-20),max(0,y1-20)
                    crop = rgb[top:min(h,y2+20),left:min(w,x2+20)]
                    result = self.model.predict(cv2.cvtColor(crop,cv2.COLOR_RGB2BGR),
                                                imgsz=640,conf=self.confidence,verbose=False)[0]
                    boxes = []
                    for box in result.boxes:
                        bx1,by1,bx2,by2 = box.xyxy[0].cpu().tolist()
                        boxes.append([bx1+left,by1+top,bx2+left,by2+top])
                for bx1,by1,bx2,by2 in boxes:
                    rect = np.array([[bx1,by1],[bx2,by1],[bx2,by2],[bx1,by2]],np.float32)
                    intersection,_ = cv2.intersectConvexConvex(polygon,rect)
                    overlap = max(overlap,intersection/max(area,1))
                raw = 'occupied' if overlap > .12 else 'free'
                previous,count = self.history.get(slot['id'],(None,0))
                count = count+1 if previous == raw else 1
                self.history[slot['id']] = (raw,count)
                # Require repeated observations; a missed frame does not immediately free a slot.
                status = raw if count >= self.confirmation_frames else 'unknown'
            else:
                self.history.pop(slot['id'],None)
            points = np.array(corners,np.float32).astype(int) if all(p is not None for p in corners) else None
            self.slots.append(dict(id=slot['id'],status=status,overlap=float(overlap),
                                   polygon=points.tolist() if points is not None else [],world=slot))
            if points is not None:
                color={'free':(40,220,80),'occupied':(255,60,60),'unknown':(255,190,0)}[status]
                cv2.polylines(output,[points],True,color,2)
                cv2.putText(output,slot['id'],tuple(points.mean(axis=0).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
        self.summary={state:[s['id'] for s in self.slots if s['status']==state] for state in ('free','occupied','unknown')}
        left, top, right, bottom = self.monitor_roi
        output = output[top:bottom, left:right].copy()
        cv2.rectangle(output,(0,0),(output.shape[1],35),(20,20,20),-1)
        cv2.putText(output,'Available %d | Occupied %d | Unknown %d' % tuple(len(self.summary[s]) for s in ('free','occupied','unknown')),
                    (10,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        return output,self.slots
