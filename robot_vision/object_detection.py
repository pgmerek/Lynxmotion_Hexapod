"""
Script to identify objects based on their color
Author: Emma Smith
"""
import numpy as np
import imutils
import time
import cv2
# from picamera.array import PiRGBArray
# from picamera import PiCamera


class MyObject:
    def __init__(self, frame, object_type, size, location):
        self.frame = frame
        self.object_type = object_type
        self.size = size
        self.location = location


MAX_RADIUS = 50
MIN_RADIUS = 0.5
SMALL = 10
MEDIUM = 30
LARGE = 50


# Define the lower and upper boundaries of the colors in the HSV color space
lower = {'red': (166, 84, 141), 'green': (66, 122, 129), 'blue': (97, 100, 117), 'yellow': (23, 59, 119)}  # assign new item lower['blue'] = (93, 10, 0)
upper = {'red': (186, 255, 255), 'green': (86, 255, 255), 'blue': (117, 255, 255), 'yellow': (54, 255, 255)}

# Define standard colors for circle around the object
colors = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0), 'yellow': (0, 255, 217),
          'orange': (0, 140, 255)}


def detect_color(image):
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(image, width=600)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # for each color in dictionary check object in frame
    for key, value in upper.items():
        # construct a mask for the color from dictionary`1, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        kernel = np.ones((9, 9), np.uint8)
        mask = cv2.inRange(hsv, lower[key], upper[key])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size. Correct this value for your obect's size
            if 0.5 < radius < 50:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 2)
                cv2.putText(frame, key, (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            colors[key], 2)

                detected_object = MyObject(frame, key, radius, center)
                return detected_object

    no_image = MyObject("None", "None", "None")
    return no_image


counter = 1
'''def raspi_camera():
    # camera = cv2.VideoCapture(0)
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    raw_capture = PiRGBArray(camera, size=(640, 480))

    time.sleep(0.1)

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        image = frame.array
        color = detect_color(image)
        counter += 1
        print(counter, "I see a ", color.object_type, "object, that is ", color.size, " in size and is located at ", color.location)
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        raw_capture.truncate(0)
    cv2.destroyAllWindows()
'''


def laptop_camera():
    camera = cv2.VideoCapture(0)
    while True:
        (grabbed, frame) = camera.read()
        color = detect_color(frame)
        print(counter, "I see a ", color.object_type, "object, that is ", color.size, " in size and is located at ", color.location)
        cv2.imshow("Frame", color.frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    camera.release()
    cv2.destroyAllWindows()


