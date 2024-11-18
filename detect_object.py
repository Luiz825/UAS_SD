import cv2
from ultralytics import YOLO
import math

model = YOLO("/home/dronelyraven/2024_SD/UAS_SD/runs /detect/train6/weights/best.pt")

classname = ["bulls eye"]

cam = cv2.VideoCapture(0)

#cam.set(3,320)
#cam.set(4,320)

while True:
    ret, frame = cam.read()

    frame_resized = cv2.resize(frame, (320,320))
    results = model(frame_resized, imgsz=320,stream=True)

    # coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

            # put box in cam
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)

            # class name
            cls = int(box.cls[0])
            print("Class name -->", classname[cls])

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(frame_resized, classname[cls], org, font, fontScale, color, thickness)

    cv2.imshow('Webcam', frame_resized)
    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()