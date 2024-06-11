#!/usr/bin/python3

import cv2

# Global variable to store the current mouse position
current_x, current_y = -1, -1

# Function to get the RGB values of a pixel at (x, y)
def get_pixel_rgb(image, x, y):
    # OpenCV uses BGR format by default
    b, g, r = image[y, x]
    return (r, g, b)

# Mouse callback function to update the coordinates
def mouse_callback(event, x, y, flags, param):
    global current_x, current_y
    if event == cv2.EVENT_MOUSEMOVE:  # Mouse move event
        current_x, current_y = x, y

# Initialize the webcam
cap = cv2.VideoCapture(0)

cv2.namedWindow('Frame')
cv2.setMouseCallback('Frame', mouse_callback)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Check if the coordinates are valid
    if 0 <= current_x < frame.shape[1] and 0 <= current_y < frame.shape[0]:
        # Get RGB values of the pixel
        rgb_values = get_pixel_rgb(frame, current_x, current_y)
        print(f"RGB values at ({current_x}, {current_y}): {rgb_values}")

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Wait for a key press and check if it's 'q' to quit
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()