from ultralytics import YOLO

model = YOLO("/home/dronelyraven/2024_SD/UAS_SD/runs /detect/train6/weights/best.pt")
model.export(format='onnx', dynamic=True)