#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#!/usr/bin/env python

import rospy
import torch
import pyttsx3
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

# Initialize the model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='path/to/best.pt')  # Update the path to your trained model

# Initialize the speech synthesis engine
engine = pyttsx3.init()

# Initialize the CvBridge
bridge = CvBridge()

# Define the function for fruit detection
def detect_fruits(cv_image):
    # Convert the OpenCV image to PIL format
    img = Image.fromarray(cv_image)
    
    # Perform inference
    results = model(img)

    # Extract the labels and coordinates
    labels, coords = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

    # Process detection results
    fruits = {}
    for label in labels:
        fruit_name = results.names[int(label)]
        if fruit_name in fruits:
            fruits[fruit_name] += 1
        else:
            fruits[fruit_name] = 1

    return fruits

# Define the function for speech synthesis
def announce_fruits(fruits):
    announcement = ""
    for fruit, count in fruits.items():
        announcement += f"There are {count} {fruit}s. " if count > 1 else f"There is one {fruit}. "
    
    engine.say(announcement)
    engine.runAndWait()

# Define the image callback function
def image_callback(msg):
    # Convert the ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    # Detect fruits
    fruits = detect_fruits(cv_image)
    
    # Announce the fruits
    if fruits:
        announce_fruits(fruits)
    else:
        engine.say("No fruits detected.")
        engine.runAndWait()

    # Publish detected fruits to a ROS topic
    fruit_list = ', '.join([f"{count} {fruit}(s)" for fruit, count in fruits.items()])
    fruit_pub.publish(fruit_list)

if __name__ == "__main__":
    rospy.init_node('fruit_detection_node', anonymous=True)

    # Create a subscriber for the USB camera
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    # Create a publisher for detected fruits
    fruit_pub = rospy.Publisher('/detected_fruits', String, queue_size=10)

    rospy.spin()

