from picamera2 import Picamera2
import cv2
import time

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# Initialize time variables to calculate FPS manually
prev_time = time.time()

while True:
    frame = picam2.capture_array()
    # Calculate FPS manually by using time difference
    current_time = time.time()
    time_diff = current_time - prev_time
    prev_time = current_time

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Display FPS on the frame
    fps_display = 1 / time_diff
    fps_display = round(fps_display, 2)

    # Put FPS text on the frame
    cv2.putText(frame, f"FPS: {fps_display}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

    cv2.imshow("cam feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
