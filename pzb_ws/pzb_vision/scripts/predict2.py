#!/usr/bin/env python3

from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO('/home/max/irs/src/TurboPlusPlus_ImplementacionRobotica/pzb_ws/pzb_vision/scripts/runs/detect/train_tiles/weights/best.pt')  # pretrained YOLOv8n model

# Define path to video file
source = 'ducks.mp4'

# Run inference on the source
results = model(source, stream=True)  # generator of Results objects

# Process results generator
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename='result.jpg')  # save to disk