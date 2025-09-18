# Importing OpenCV Library for basic image processing functions
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO  # Import Library to access GPIO PIN

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
buzzer_pin = 36  # Define PIN for buzzer
LED_PIN = 29  # Define PIN for LED
GPIO.setup(buzzer_pin, GPIO.OUT)  # Set pin function as output
GPIO.setup(LED_PIN, GPIO.OUT)  # Set pin function as output

# Load OpenCV Haar Cascades for face and eye detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')


# Function to calculate EAR (Eye Aspect Ratio)
def compute_ear(eye_points):
    A = np.linalg.norm(eye_points[1] - eye_points[5])
    B = np.linalg.norm(eye_points[2] - eye_points[4])
    C = np.linalg.norm(eye_points[0] - eye_points[3])
    ear = (A + B) / (2.0 * C)
    return ear


# Initialize Camera
cap = cv2.VideoCapture(0)

# Status tracking
sleep = 0
drowsy = 0
active = 0
status = ""
color = (0, 0, 0)

# Set the EAR threshold and consecutive frame threshold
EAR_THRESHOLD = 0.25
CONSECUTIVE_FRAMES = 2

while True:
    _, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x, y, w, h) in faces:
        face_region = frame[y:y + h, x:x + w]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Detect eyes in the face region
        eyes = eye_cascade.detectMultiScale(face_region)
        if len(eyes) >= 2:  # Ensure both eyes are detected
            left_eye = eyes[0]
            right_eye = eyes[1]

            # Get the coordinates for both eyes (simplified)
            left_eye_points = np.array([(x + left_eye[0], y + left_eye[1]),
                                        (x + left_eye[0] + left_eye[2], y + left_eye[1]),
                                        (x + left_eye[0] + left_eye[2], y + left_eye[1] + left_eye[3]),
                                        (x + left_eye[0], y + left_eye[1] + left_eye[3])])
            right_eye_points = np.array([(x + right_eye[0], y + right_eye[1]),
                                         (x + right_eye[0] + right_eye[2], y + right_eye[1]),
                                         (x + right_eye[0] + right_eye[2], y + right_eye[1] + right_eye[3]),
                                         (x + right_eye[0], y + right_eye[1] + right_eye[3])])

            # Calculate EAR for both eyes
            left_ear = compute_ear(left_eye_points)
            right_ear = compute_ear(right_eye_points)

            # Check for blinking or drowsiness based on EAR
            if left_ear < EAR_THRESHOLD and right_ear < EAR_THRESHOLD:
                sleep += 1
                drowsy = 0
                active = 0
                if sleep > CONSECUTIVE_FRAMES:
                    status = "SLEEPING !!!"
                    GPIO.output(buzzer_pin, GPIO.HIGH)
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    color = (0, 0, 255)  # Red for sleep warning
            elif left_ear < EAR_THRESHOLD or right_ear < EAR_THRESHOLD:
                sleep = 0
                drowsy += 1
                active = 0
                if drowsy > CONSECUTIVE_FRAMES:
                    status = "Drowsy!"
                    GPIO.output(buzzer_pin, GPIO.HIGH)
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    color = (0, 0, 255)  # Red for drowsiness warning
            else:
                drowsy = 0
                sleep = 0
                active += 1
                if active > CONSECUTIVE_FRAMES:
                    status = "Active :)"
                    GPIO.output(buzzer_pin, GPIO.LOW)
                    GPIO.output(LED_PIN, GPIO.LOW)
                    color = (0, 255, 0)  # Green for active

        # Display status text on the frame
        cv2.putText(frame, status, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

    # Show the video frame
    cv2.imshow("Driver Monitoring", frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()