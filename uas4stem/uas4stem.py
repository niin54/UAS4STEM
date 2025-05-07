import cv2
import numpy as np
from dronekit import connect, VehicleMode

# Load the predetermined set of 20 images
images = []
for i in range(20):
    img = cv2.imread(f"image_{i}.jpg")
    images.append(img)

# Connect to the drone
vehicle = connect('/dev/ttyUSB0', wait_ready=True)

# Define a function to identify the image in the camera feed
def identify_image(frame):
    for img in images:
        result = cv2.matchTemplate(frame, img, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        if max_val > 0.8:  # adjust the threshold value as needed
            return max_loc
    return None

# Initialize the camera feed
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Identify the image in the frame
    loc = identify_image(frame)
    if loc is not None:
        print(f"Image identified at location {loc}")
        # Move the drone to center the camera feed over the identified image
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        vehicle.channels.overrides = {'3': 1500}  # set throttle to 1500
        vehicle.simple_goto((loc[0], loc[1], 0))
    else:
        print("No image identified")

    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
vehicle.close()