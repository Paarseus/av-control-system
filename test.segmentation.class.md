# RealSense Segmentation for Autonomous Vehicles

Simple Python wrapper around a lane/road/object segmentation stack for Intel RealSense cameras or plain images. Designed for quick bring-up on Jetson Nano or any Linux box with a GPU/CPU.

## Requirements

- Python 3.8+
- Intel RealSense camera (optional if you only process saved images)
- Weights:
  - `data/weights/yolopv2.pt` for road/lane segmentation
  - `yolov8m-seg.pt` (or `yolov8n-seg.pt`) for object segmentation

## Installation

```bash
pip install -r requirements.txt
# or install directly
pip install opencv-python numpy torch torchvision ultralytics pyrealsense2
```

## Quickstart

**Single image:**
```python
from realsense_segmenter import RealSenseSegmenter
import cv2

segmenter = RealSenseSegmenter()
image = cv2.imread("road.jpg")
segmented = segmenter.segment(image)
cv2.imwrite("output.jpg", segmented)
```

**Live RealSense stream:**
```python
from realsense_segmenter import RealSenseSegmenter
import cv2

segmenter = RealSenseSegmenter()
segmenter.connect_camera(width=640, height=480, fps=30)

while True:
    frame = segmenter.get_segmented_frame(show_masks=True, show_boxes=False)
    if frame is None:
        break
    cv2.imshow("Segmentation", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

segmenter.disconnect()
```

**Example: record segmented frames to disk (camera or image list):**
```python
from pathlib import Path
import cv2
from realsense_segmenter import RealSenseSegmenter

segmenter = RealSenseSegmenter(conf_threshold=0.3)
segmenter.connect_camera()
out_dir = Path("segmented_samples")
out_dir.mkdir(exist_ok=True)

for idx in range(30):  # grab 30 frames
    frame = segmenter.get_segmented_frame(show_masks=True, show_boxes=True)
    if frame is None:
        break
    cv2.imwrite(out_dir / f"frame_{idx:03d}.png", frame)

segmenter.disconnect()
```

## What You Get

- **Lane detection** — red overlay on lane lines
- **Drivable area** — green overlay on the road surface
- **Object segmentation** — colored masks (and optional boxes) for vehicles, pedestrians, bicycles, traffic lights
- **Structured data** — masks, object list, and lane metrics for downstream control

## API Cheatsheet

- `RealSenseSegmenter(conf_threshold=0.25)`: initialize the wrapper
- `connect_camera(width=640, height=480, fps=30)`: start RealSense stream
- `get_segmented_frame(show_masks=True, show_boxes=False)`: grab + segment a live frame
- `segment(image, show_masks=True, show_boxes=False)`: run segmentation on a BGR image
- `get_data(image)`: return masks, objects, and vehicle metrics without visualization
- `disconnect()`: stop the camera pipeline

## Files

- `realsense_segmenter.py` — public-facing wrapper and camera helpers
- `segmentation_engine.py` — model loading, inference, visualization, and metrics
- `utils/utils.py` — helpers (device selection, preprocessing, masks)
- `data/weights/` — model weights folder

## Output Data Example

```python
data = segmenter.get_data(image)

print(data.keys())
# dict keys: 'lanes', 'road', 'objects', 'vehicle_data'

print(data['vehicle_data'])
# {
#   'lane_left_x': int or None,
#   'lane_right_x': int or None,
#   'lane_center_x': int or None,
#   'lane_deviation': int,
#   'drivable_area_pct': float,
#   'vehicle_count': int,
#   'pedestrian_count': int,
#   'bicycle_count': int,
#   'traffic_light_count': int,
#   'closest_obstacle_dist': int or None,
#   'frame_width': int,
#   'frame_height': int
# }
```

## Tips

- Run on GPU when available (`device` selection handled inside `segmentation_engine.py`).
- Lower `img_size` and/or `conf_threshold` for faster FPS if you edit `SegmentationEngine` defaults.
- Verify weight paths before running; missing weights are logged on startup.

## License

See LICENSE file.
