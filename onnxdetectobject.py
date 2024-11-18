import cv2
import numpy as np
import onnxruntime as ort
import math

# Path to the exported ONNX model
model_path = "/home/dronelyraven/2024_SD/UAS_SD/runs /detect/train6/weights/best.onnx"

# Initialize ONNX Runtime session
session = ort.InferenceSession(model_path)

# Class names (replace with your own class names)
classname = ["bulls eye"]

# Open webcam (or use another video source)
cam = cv2.VideoCapture(0)

# Check if webcam is opened successfully
if not cam.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cam.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Resize image to 224x224 (or the size your model expects)
    frame_resized = cv2.resize(frame, (224, 224))

    # Convert image to RGB and normalize
    frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
    frame_rgb = np.transpose(frame_rgb, (2, 0, 1))  # Convert to CxHxW
    frame_rgb = frame_rgb.astype(np.float32)  # Convert to float32
    frame_rgb /= 255.0  # Normalize to [0, 1]

    # Prepare input data for the model (batch_size, channels, height, width)
    input_data = np.expand_dims(frame_rgb, axis=0)  # Add batch dimension

    # Run inference with ONNX model
    inputs = {session.get_inputs()[0].name: input_data}
    outputs = session.run(None, inputs)

    # Post-process the model's output (e.g., bounding boxes, class, confidence)
    # Assuming the output format is (batch_size, num_boxes, 6) (x1, y1, x2, y2, confidence, class)
    boxes = outputs[0]

    for box in boxes:
        
        # Unpack the 5 values from the box: x1, y1, x2, y2, and confidence
        x1, y1, x2, y2, conf = box  # Assuming box.xyxy contains the 5 values

        # Convert to int for drawing the bounding box
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # Calculate confidence and print it
        confidence = math.ceil((conf * 100)) / 100
        print("Confidence --->", confidence)

        # Class index (you may need to access it differently, depending on the model output)
        cls = int(box.cls[0])  # Assuming 'box.cls' contains the class index

        # Print the class name
        print("Class name -->", classname[cls])

        # Draw the bounding box and class label
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
        org = [x1, y1]
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (255, 0, 0)
        thickness = 2
        cv2.putText(frame, classname[cls], org, font, fontScale, color, thickness)

    # Display the frame
    cv2.imshow('Webcam', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) == ord('q'):
        break

# Release the webcam and close windows
cam.release()
cv2.destroyAllWindows()
