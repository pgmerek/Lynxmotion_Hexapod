"""
Script to identify objects based on their color
Author: Emma Smith
"""
import numpy as np
import imutils
import time
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera


class MyObject:
    """
    Generic object
    """
    def __init__(self, frame, object_type, size, location):
        self.frame = frame
        self.object_type = object_type
        self.size = size
        self.location = location


# Parameters to define frame and object
FRAME_SIZE = 600
MAX_RADIUS = 80
MIN_RADIUS = 0.5
FRAME_RATE =  60
RESOLUTION = (600, 400)

# Parameters to define size and location of object
LEFT = FRAME_SIZE/3
RIGHT = 2 * FRAME_SIZE/3
SMALL = (MAX_RADIUS * MAX_RADIUS/3) * np.pi
LARGE = 2 * (MAX_RADIUS* MAX_RADIUS/3) * np.pi


# Define the lower and upper boundaries of the colors in the HSV color space
lower = {'blue': (97, 100, 117), 'yellow': (0, 135, 184), 'pink': (96, 20, 199)}

upper = {'blue': (117, 255, 255), 'yellow': (54, 255, 255), 'pink':(238, 192, 255)}

# Define standard colors for circle around the object
colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0), 'yellow': (0, 255, 217),
        'orange': (0, 140, 255), 'pink':(147, 20, 255)}


def detect_color(image):
    """
    Detect objects by color in image
    :param image: image to detect objects from
    :return: returns the processed image, color, size, and location
    """
    # Resize the frame, blur it, and convert it to the HSV color space
    frame = imutils.resize(image, width=FRAME_SIZE)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # For each color in dictionary check object in frame
    for key, value in upper.items():
        # Construct a mask for the color from dictionary`1, then perform
        # A series of dilations and erosions to remove any small
        # blobs left in the mask
        kernel = np.ones((9, 9), np.uint8)
        mask = cv2.inRange(hsv, lower[key], upper[key])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        # only proceed if at least one contour was found
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size. Correct this value for your obect's size
            if MIN_RADIUS < radius < MAX_RADIUS:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 2)
                cv2.putText(frame, key + str(radius), (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            colors[key], 2)

                object_size = np.pi * radius * radius
                if object_size <= SMALL:
                    object_size = "small"
                elif object_size >= LARGE:
                    object_size = "large"
                else:
                    object_size = "medium"

                object_location = "Middle"
                if center[0] <= LEFT:
                    object_location = "left"
                elif center[0] >= RIGHT:
                    object_location = "right"
                detected_object = MyObject(frame, key, object_size, object_location)
                return detected_object

    no_image = MyObject(frame, "None", "None", "None")
    return no_image

def detect_face(image):
    """
    detects faces in the image
    :param image: 
    :return: returns object
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_casca



def raspi_camera():
    """
    Use this function when running this on the pi
    """
    # camera = cv2.VideoCapture(0)
    camera = PiCamera()
    camera.resolution = RESOLUTION
    camera.framerate = FRAME_RATE
    raw_capture = PiRGBArray(camera, size=RESOLUTION)

    time.sleep(0.1)

    counter = 1
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        image = frame.array
        color = detect_color(image)
        counter += 1
        print(counter, "I see a ", color.object_type, "object, that is ", color.size, " in size and is located at ", color.location)
        cv2.imshow("Frame", color.frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        raw_capture.truncate(0)
    cv2.destroyAllWindows()



def laptop_camera():
    """
    Use this function when running on a laptop
    :return: nothing
    """
    counter = 1
    camera = cv2.VideoCapture(0)
    # Grab frame and process until user presses "q"
    while True:
        (grabbed, frame) = camera.read()
        color = detect_color(frame)
        print(counter, "I see a ", color.object_type, "object, that is ", color.size, " in size and is located at ", color.location)
        counter += 1
        cv2.imshow("Frame", color.frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    camera.release()
    cv2.destroyAllWindows()


# laptop_camera()
raspi_camera()
