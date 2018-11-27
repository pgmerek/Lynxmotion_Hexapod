#!/usr/bin/env python

# Import Libraries 
import cv2 
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
import time
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import imutils
import numpy as np

class detected_object:
    """
    Generic object
    """
    def __init__(self, color, size, location):
        self.color = color 
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
global pi
global done

vision = 1
pi = 0 
done = 0

#node to pulish
vision_publisher = rospy.Publisher('vision_return', String, queue_size=1)
vision_publisher_finished = rospy.Publisher('vision_finished', Int32, queue_size=1)

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

def vision_callback(data):
    global vision
    print ("I received a ")
    print (data.data)
    vision = data.data
    if vision and pi:
        raspi_camera()
    elif vision and not pi:
        test_loop()
    else:
        time.sleep(1)

def eyes_main():
    global vision 
    global pi
    global done
    rospy.init_node("eyes", anonymous=True)
    rate = rospy.Rate(10) # publish 10 times a second

    rospy.Subscriber('vision_command', Int32, vision_callback)
        
    rospy.spin()


def test_loop():
    global done
    global vision
    count = 0
    while count <= 10:
        vision_publisher.publish("test test test")
        count += 1
    done += 1
    vision_publisher_finished.publish(done)


def raspi_camera():
    """
    Use this function when running this on the pi
    """
    global pi
    global vision
    counter = 0

    # Initialize Camera and start grabbing frames
    camera = PiCamera()
    camera.resolution = RESOLUTION
    camera.framerate = FRAME_RATE
    raw_capture = PiRGBArray(camera, size=RESOLUTION)
    time.sleep(0.1) # time to grab a frame

    # Process 5 frames and publish data
    while counter < 5:
        with camera.capture_continuous(raw_capture, format="bgr", use_video_port=True) as frame:
            image = frame.array
            item = detect_color(image)
            # publish string in "{color} {location} {size}}" format
            published_data = "{0} {1} {2}".format(item.color, item.location, item.size)
            vision_publisher.publish(published_data)
            raw_capture.truncate(0)
            # may not need this. time to grab frame
            time.sleep(0.05)
            counter += 1
    # done processing frames. Will wait for next vision request
    done += 1
    camera.close()
    vision_publisher_finished.publish(done)
    
if __name__ == '__main__':
    print("My eyes are working")
    try:
        eyes_main()
    except rospy.ROSInterruptException():
        pass

