import cv2
import time

# Open the default camera
cam = cv2.VideoCapture(0)

# Check if the camera is opened successfully
if not cam.isOpened():
    print("Error: Could not open camera.")
    exit()

# Get the frame rate of the camera
fps = cam.get(cv2.CAP_PROP_FPS)

# Initialize time variables to calculate FPS manually
prev_time = time.time()

while True:
    ret, frame = cam.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Calculate FPS manually by using time difference
    current_time = time.time()
    time_diff = current_time - prev_time
    prev_time = current_time

    # Display FPS on the frame
    fps_display = 1 / time_diff
    fps_display = round(fps_display, 2)

    # Put FPS text on the frame
    cv2.putText(frame, f"FPS: {fps_display}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    # Display the captured frame
    cv2.imshow('Camera', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()

# Print the FPS value retrieved from the camera (if available)
print(f"Camera FPS: {fps}")
