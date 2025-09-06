"""
LAB Mask Range Tuner for Color Detection

This script helps you interactively find the correct LAB color range for masking a color in a camera feed.
It uses OpenCV to display a live feed and a region-of-interest (ROI) mask, with sliders to adjust LAB range.
When you press 'q', the selected LAB bounds are printed for use in your vision code.

Usage:
- Run the script on your Raspberry Pi or computer with a USB camera.
- Move the L, A, B sliders to isolate the target color in the ROI.
- The percentage of detected pixels in the ROI is shown.
- Use the final lower/upper LAB values in your project (e.g., for `masks.py`).

"""

import cv2
import numpy as np

# --- Trackbar callback (does nothing, needed for createTrackbar) ---
def nothing(x):
    pass

# Open USB camera (index 0)
cap = cv2.VideoCapture(0)

# ROI coordinates and size (x, y, width, height)
roi_x, roi_y, roi_w, roi_h = 200, 150, 200, 200   # Adjust for your scene

# Create a window with LAB sliders
cv2.namedWindow("Trackbars")
# Lower LAB bounds
cv2.createTrackbar("L1", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("A1", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("B1", "Trackbars", 0, 255, nothing)
# Upper LAB bounds
cv2.createTrackbar("L2", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("A2", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("B2", "Trackbars", 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Extract ROI from frame
    roi = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

    # Convert ROI to LAB color space
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

    # Get slider (trackbar) positions for LAB lower and upper bounds
    l1 = cv2.getTrackbarPos("L1", "Trackbars")
    a1 = cv2.getTrackbarPos("A1", "Trackbars")
    b1 = cv2.getTrackbarPos("B1", "Trackbars")
    l2 = cv2.getTrackbarPos("L2", "Trackbars")
    a2 = cv2.getTrackbarPos("A2", "Trackbars")
    b2 = cv2.getTrackbarPos("B2", "Trackbars")

    lower = np.array([l1, a1, b1], dtype=np.uint8)
    upper = np.array([l2, a2, b2], dtype=np.uint8)

    # Apply LAB mask to the ROI
    mask = cv2.inRange(lab, lower, upper)

    # Calculate percentage of masked pixels in ROI
    detected_pixels = cv2.countNonZero(mask)
    total_pixels = roi_w * roi_h
    percent = (detected_pixels / total_pixels) * 100

    # Draw bounding box and percentage reading on main frame
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 0), 2)
    cv2.putText(frame, f"Detected: {percent:.1f}%", (roi_x, roi_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show live video and mask
    cv2.imshow("Live Feed", frame)
    cv2.imshow("ROI Mask", mask)  # Mask view for tuning

    # Press 'q' to quit and print final LAB values
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Final Range:")
        print("Lower:", lower.tolist())
        print("Upper:", upper.tolist())
        break

# Release camera and close windows
cap.release()
cv2.destroyAllWindows()