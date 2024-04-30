#!/usr/bin/env python3

from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n.yaml')  # build a new model from YAML

# Train the model
results = model.train(
    data='/home/max/irs/src/TurboPlusPlus_ImplementacionRobotica/pzb_ws/pzb_vision/scripts/floor_tiles/data.yaml', 
    epochs=200, imgsz=640)
