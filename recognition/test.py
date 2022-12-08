import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np
import math
from tensorflow import keras
from PIL import Image, ImageOps

CONFIDENT_CONSEC_THRESHOLD = 5

def main():
    current_index = 0
    consecutive_count = 0

    np.set_printoptions(suppress=True)

    # Load the model
    model = keras.models.load_model('model/keras_Model.h5', compile=False)
    # Load the labels
    class_names = open('model/labels.txt', 'r').readlines()
    data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)

    # Video capture on the primary camera, 0
    cap = cv2.VideoCapture(0)
    # Only want to detect one hand
    detector = HandDetector(maxHands=1)

    # width and height of photos
    dimension = 400

    while True:
        # capture the the frame and report the success on this capture
        captured, image = cap.read()

        # Get the data about the hand (locations of indexes, width and heights), and capture the image again, outlining the hand
        hands, image= detector.findHands(image)

        image_copy = image

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

            # The following block of code was mostly provided by teachable machine with google, as they were demonstrating how to use their model

            image = Image.fromarray(backgroundImage)
            #resize the image to a 224x224 with the same strategy as in TM2:
            #resizing the image to be at least 224x224 and then cropping from the center
            size = (224, 224)
            image = ImageOps.fit(image, size, Image.Resampling.LANCZOS)
            #turn the image into a numpy array
            image_array = np.asarray(image)
            # Normalize the image
            normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1
            # Load the image into the array
            data[0] = normalized_image_array

            # run the inference
            prediction = model.predict(data)
            index = np.argmax(prediction)
            class_name = class_names[index]
            confidence_score = prediction[0][index]

            # This ends the block of code provided by teachable machine

            print("Destination Number: ", index)

            if index == current_index:
                consecutive_count += 1
            else:
                consecutive_count = 1
                current_index = index
            if consecutive_count >= CONFIDENT_CONSEC_THRESHOLD:
                return current_index
            cv2.imshow("Full image", backgroundImage)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()