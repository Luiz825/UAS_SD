from ultralytics import YOLO

#load model
model = YOLO("/home/luiz/UAS_SD/runs/detect/train_new/weights/best.pt")


#start training
results = model.train(data="config.yaml", epochs=100)