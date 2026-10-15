import os
os.environ["TRANSFORMERS_OFFLINE"] = "1"

import numpy as np
import cv2
import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation

class DepthEstimator:
    def __init__(self, threshold=0.15):
        print("Loading Depth Anything V2...")
        self.processor = AutoImageProcessor.from_pretrained(
            "depth-anything/Depth-Anything-V2-Small-hf")
        self.model = AutoModelForDepthEstimation.from_pretrained(
            "depth-anything/Depth-Anything-V2-Small-hf")
        self.model.eval()
        self.threshold = threshold
        self.trigger_count = 0
        print("Depth model ready")

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

        # Draw detection zone on visualisation
        h, w = depth.shape
        cv2.rectangle(depth_vis,
                      (w//6, h//4),
                      (5*w//6, 3*h//4),
                      (0, 255, 0), 2)

        return depth, depth_vis

    def obstacle_ahead(self, depth_map):
        h, w = depth_map.shape

        # Centre half of frame for detection
        centre = depth_map[h//4:3*h//4, w//6:5*w//6]

        # Use 5th percentile, more robust than minimum
        p5 = float(np.percentile(centre, 5))
        min_depth = float(centre.min())

        print(f"DEPTH: p5={p5:.3f} min={min_depth:.3f} threshold={self.threshold}")

        if p5 < self.threshold:
            self.trigger_count += 1
            print(f"DEPTH: trigger count {self.trigger_count}/2")
            if self.trigger_count >= 2:
                print("DEPTH: obstacle confirmed")
                self.trigger_count = 0
                return True
        else:
            self.trigger_count = 0

        return False

    def get_detection_distance(self, depth_map, bbox):
        x1, y1, x2, y2 = [int(v) for v in bbox]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        h, w = depth_map.shape
        cx = min(cx, w - 1)
        cy = min(cy, h - 1)
        return float(depth_map[cy, cx])