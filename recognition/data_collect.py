import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np
import math
import time

# Video capture on the primary camera, 0
cap = cv2.VideoCapture(0)
# Only want to detect one hand
detector = HandDetector(maxHands=1)

# width and height of photos
dimension = 400

# For the purposes of getting photos, commented out now
# counter = 0
# folder = "data/C"


while True:
    # capture the the frame and report the success on this capture
    captured, image = cap.read()

    # Get the data about the hand (locations of indexes, width and heights), and capture the image again, outlining the hand
    hands, image= detector.findHands(image)

    # if hands are detected
    if hands:
        # only one hand
        hand = hands[0]

        # some info from the hand
        x, y, w, h = hand['bbox']

        # Make a background white image, with a unit8 to store numbers up to 255, multiply by 255 to make it white
        backgroundImage = np.ones((dimension, dimension, 3), np.uint8)*255 

        # Crop it so it's just the hand in the image, using the hand dimensions
        croppedImage = image[y - 25: y+h + 25, x  - 25: x+w + 25]

        # Taller than it is wide
        if h > w:
            multiplier_constant = dimension/h
            width = math.floor(multiplier_constant*w)
            resizedImage = cv2.resize(croppedImage, (width, dimension))
            gap = math.floor((dimension - width) / 2)
            backgroundImage[0:dimension, gap:width+gap] = resizedImage
        # Wider than it is tall
        else: 
            multiplier_constant = dimension/w
            height = math.floor(multiplier_constant*h)
            resizedImage = cv2.resize(croppedImage, (dimension, height))
            gap = math.floor((dimension - height) / 2)
            backgroundImage[gap:height+gap, 0:dimension] = resizedImage

        cv2.imshow("Full image", backgroundImage)
    key = cv2.waitKey(1)

    # take photos for dataset, commented now
    # if key == ord("a"):
    #     cv2.imwrite(f'{folder}/Image_{counter}.jpg', backgroundImage)
    #     print(counter)
    #     counter += 1