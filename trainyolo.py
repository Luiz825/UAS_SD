from ultralytics import YOLO

#load model
model = YOLO("yolov8n.yaml")


#start training
results = model.train(data="config.yaml", epochs=3)