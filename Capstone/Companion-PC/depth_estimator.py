import os
os.environ["TRANSFORMERS_OFFLINE"] = "1"

import numpy as np
import cv2
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

# Zone checked for obstacles (fractions of the frame)
ZONE_Y0, ZONE_Y1 = 0.25, 0.50   # shifted up to avoid floor
ZONE_X0, ZONE_X1 = 0.40, 0.60   # narrower, closer to the drone's width
OBSTACLE_PIXEL_FRACTION = 0.20  # share of the box that must be "close"


class DepthEstimator:
    # NOTE: for this setup LARGER depth values mean CLOSER.
    # Obstacle = high value. Open space = low value.
    def __init__(self, threshold=0.6):
        print("Loading Depth Anything V2...")
        self.processor = AutoImageProcessor.from_pretrained(
            "depth-anything/Depth-Anything-V2-Small-hf")
        self.model = AutoModelForDepthEstimation.from_pretrained(
            "depth-anything/Depth-Anything-V2-Small-hf")
        self.model.eval()
        self.threshold = threshold
        self.trigger_count = 0
        self.last_p95 = 0.0
        self.frame_count = 0
        print("Depth model ready")

    def _centre(self, depth_map):
        h, w = depth_map.shape
        return depth_map[
            int(h * ZONE_Y0):int(h * ZONE_Y1),
            int(w * ZONE_X0):int(w * ZONE_X1)
        ]

    def estimate(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)
        depth = outputs.predicted_depth.squeeze().numpy()
        depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]))

        # Normalise for display only
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)

        # Detection zone + live readout
        h, w = depth.shape
        cv2.rectangle(depth_vis,
                      (int(w * ZONE_X0), int(h * ZONE_Y0)),
                      (int(w * ZONE_X1), int(h * ZONE_Y1)),
                      (0, 255, 0), 2)
        p95 = float(np.percentile(self._centre(depth), 95))
        frac = float((self._centre(depth) > self.threshold).mean())
        colour = (0, 0, 255) if frac > OBSTACLE_PIXEL_FRACTION else (0, 255, 0)
        cv2.putText(depth_vis,
                    f"frac={frac:.2f} p90={float(np.percentile(self._centre(depth), 90)):.2f} "
                    f"thr={self.threshold:.2f} trig={self.trigger_count}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

        return depth, depth_vis

    def obstacle_ahead(self, depth_map):
        centre = self._centre(depth_map)
        frac = float((centre > self.threshold).mean())
        self.last_p95 = float(np.percentile(centre, 90))

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            print(f"DEPTH: close_frac={frac:.2f} thr={self.threshold}")

        if frac > OBSTACLE_PIXEL_FRACTION:
            self.trigger_count += 1
            if self.trigger_count >= 2:
                print(f"DEPTH: obstacle confirmed frac={frac:.2f}")
                return True
        else:
            if self.trigger_count > 0:
                self.trigger_count -= 1
        return False

    def get_centre_depth(self, depth_map):
        # Raw reading for direction scanning, does not touch trigger_count
        return float(np.percentile(self._centre(depth_map), 90))

    def get_detection_distance(self, depth_map, bbox):
        x1, y1, x2, y2 = [int(v) for v in bbox]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        h, w = depth_map.shape
        cx = min(max(cx, 0), w - 1)
        cy = min(max(cy, 0), h - 1)
        return float(depth_map[cy, cx])