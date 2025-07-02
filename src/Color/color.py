import cv2
import numpy as np

# Open the CSI camera using OpenCV V4L2 driver
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Red mask (red is around 0 or 180 hue, so we combine two ranges)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # --- Green mask
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Apply masks
    res_red = cv2.bitwise_and(frame, frame, mask=mask_red)
    res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

    # Show original and masks
    cv2.imshow("Original Feed", frame)
    cv2.imshow("Red Objects", res_red)
    cv2.imshow("Green Objects", res_green)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
