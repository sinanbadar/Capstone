import numpy as np
import cv2
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import torch

class DepthEstimator:
    def __init__(self, threshold=0.3):
        print("Loading Depth Anything V2...")
        self.processor = AutoImageProcessor.from_pretrained(
            "depth-anything/Depth-Anything-V2-Small-hf")
        self.model = AutoModelForDepthEstimation.from_pretrained(
            "depth-anything/Depth-Anything-V2-Small-hf")
        self.model.eval()
        self.threshold = threshold
        print("Depth model ready")

    def estimate(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt")
        with torch.no_grad():
            outputs = self.model(**inputs)
        depth = outputs.predicted_depth.squeeze().numpy()
        depth = cv2.resize(depth, (frame.shape[1], frame.shape[0]))

        # Normalise for display
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)

        return depth, depth_vis

    def obstacle_ahead(self, depth_map):
        h, w = depth_map.shape
        centre = depth_map[h//4:3*h//4, w//4:3*w//4]
        min_depth = float(centre.min())
        max_depth = float(centre.max())
        print(f"DEPTH: min={min_depth:.3f} max={max_depth:.3f} threshold={self.threshold}")
        return min_depth < self.threshold