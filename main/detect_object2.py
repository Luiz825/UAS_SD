import cv2
from ultralytics import YOLO
import math
from picamera2 import Picamera2

model = YOLO("/home/dronelyraven/2024_SD/UAS_SD/runs /detect/train6/weights/best_ncnn_model")

classname = ["bulls eye"]

picam2 = Picamera2()
picam2.preview_configuration.main.size = (320,320)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()


#cam.set(3,320)
#cam.set(4,320)
text = "cam"
font = cv2.FONT_HERSHEY_SIMPLEX
text_size = cv2.getTextSize( text,font, 1,2)[0]
text_y = text_size[1]+10


frame_count = 0

while True:
    frame = picam2.capture_array()
    #frame_resized = cv2.resize(frame, (320,320))
    if frame_count % 2 == 0:
        
        results = model(frame, imgsz=320)
        annotated_frame=results[0].plot()


   # text_x = annotated_frame.shape[1] - text_size[0] -10
    #text_y = text_size[1]+10

    # coordinates
    #cv2.putText(annotated_frame, text,(text_x, text_y), font, 1, (255,255,255), 2, cv2.LINE_AA)
    #cv2.putText(annotated_frame, font,1, (255,255,255), 2, cv2.LINE_AA)
    frame_count += 1
    cv2.imshow('Webcam', annotated_frame)
    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()