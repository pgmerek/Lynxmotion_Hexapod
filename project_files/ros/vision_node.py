#!/usr/bin/env python

# Import Libraries 

import cv2 
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import numpy as np

class detected_object:
    """
    Generic object
    """
    def __init__(self, type, size, location):
        self.type = type
        self.size = size
        self.location = location


# Parameters to define frame and object
FRAME_SIZE = 600
MAX_RADIUS = 80
MIN_RADIUS = 0.5
FRAME_RATE =  60
RESOLUTION = (608, 400)

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

# global
global vision

#node to pulish
publish_node = '/vision'
vision_publisher = rospy.Publisher(publish_node, String, queue_size=1)

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

                object_location = "middle"
                if center[0] <= LEFT:
                    object_location = "left"
                elif center[0] >= RIGHT:
                    object_location = "right"
                detected_object = detected_object(key, object_size, object_location)
     #           cv2.imshow("Frame", frame)
                return detected_object

    no_image = detected_object("none", "none", "none")
    # cv2.imshow("Frame", frame)
    return no_image

def detect_face(image):
    """
    detects faces in the image
    :param image: 
    :return: returns object
    """
    face_cascade = cv2.CascadeClassifier('//home//pi//opencv//opencv//data//haarcascades//haarcascade_frontal_face_default.xml')

    eye_cascade = cv2.CascadeClassifier('//home//pi//opencv//opencv//data//haarcascades//haarcascade_eye.xml')

    print (face_cascade)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = image[y:y + h, x:x + w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex, ey, ew, eh) in eyes:
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
    cv2.imshow("Test", image)

    return image

def vision_callback(data):
    global vision
    vision = data.data



def raspi_camera():
    """
    Use this function when running this on the pi
    """
    # ROS
    global vision 
    rospy.init_node("eyes", anonymous=True)
    rate = rospy.Rate(10) # publish 10 times a second

    # This should run on its own thread and update  
    rospy.Subscriber('/vision_listener', Int32, vision_callback)
    
    if vision:
        # Initialize Camera and start grabbing frames
        camera = PiCamera()
        camera.resolution = RESOLUTION
        camera.framerate = FRAME_RATE
        raw_capture = PiRGBArray(camera, size=RESOLUTION)
        time.sleep(0.1) # time to grab a frame

        # Process each frame and publish data
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            if vision:
                image = frame.array
                color = detect_color(image)
                vision_publisher.publish(color.type, color.location, color.size)
                raw_capture.truncate(0)
            else:
                break
        time.sleep(1)
    else:
        time.sleep(1)



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
        print(counter, "I see a ", color.type, "object, that is ", color.size, " in size and is located at ", color.location)
        counter += 1
        cv2.imshow("Frame", color.frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print("My eyes are working")
    try:
        laptop_camera()
    except rospy.ROSInterruptException():
        pass

# laptop_camera()
# raspi_camera()


