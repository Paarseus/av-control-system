# RealSense Segmentation for Autonomous Vehicles

Simple Python class for real-time road segmentation on Jetson Nano with Intel RealSense camera.

## Installation

```bash
pip install opencv-python numpy torch torchvision ultralytics pyrealsense2
```

## Usage

```python
from realsense_segmenter import RealSenseSegmenter
import cv2

# Initialize
segmenter = RealSenseSegmenter()

# Option 1: With RealSense camera
segmenter.connect_camera()
while True:
    segmented = segmenter.get_segmented_frame()
    cv2.imshow("Output", segmented)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
segmenter.disconnect()

# Option 2: With image file
image = cv2.imread("road.jpg")
segmented = segmenter.segment(image)
cv2.imwrite("output.jpg", segmented)
```

## What It Does

- **Lane detection** - Red overlay on lane lines
- **Drivable area** - Green overlay on road surface  
- **Object segmentation** - Color masks on vehicles, pedestrians, bicycles, traffic lights

## Files

- `realsense_segmenter.py` - Main class
- `segmentation_engine.py` - Processing engine
- `virtual_can_bus.py` - CAN bus simulator
- `show_segmentation.py` - Visualization tool

## Test It

```bash
# Process an image
python show_segmentation.py your_image.jpg

# Results saved to segmentation_examples/ folder
```

## Output Data

```python
data = segmenter.get_data(image)

# Returns:
# - data['lanes'] - Binary mask of lane lines
# - data['road'] - Binary mask of drivable area
# - data['objects'] - List of detected objects
# - data['vehicle_data'] - Lane positions, deviation, object counts
```

## Models

- YOLOPv2 for road/lane segmentation
- YOLOv8 for object detection

Place weights in:
- `data/weights/yolopv2.pt`
- `yolov8m-seg.pt`

## Performance

- ~30 FPS on Jetson Nano with GPU
- Lower resolution for faster processing: `RealSenseSegmenter(img_size=416)`

## License

See LICENSE file
