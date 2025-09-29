# -*- coding: utf-8 -*-

import cv2
from flask import Flask, render_template, Response
import threading

app = Flask(__name__)
cap = cv2.VideoCapture('http://192.168.8.229:8081/')
cap_lock = threading.Lock()

def video_capture_thread():
    global cap
    while True:
        with cap_lock:
            ret, frame = cap.read()
        if not ret:
            with cap_lock:
                cap = cv2.VideoCapture("http://192.168.8.229:8081/")
            continue

def person_detection_thread():
    global cap
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    while True:
        with cap_lock:
            ret, frame = cap.read()
        if not ret:
            continue

        # Resize frame to speed up detection
        frame = cv2.resize(frame, (640, 480))

        # Convert the frame to grayscale for HOG
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect people in the frame
        boxes, weights = hog.detectMultiScale(gray, winStride=(8, 8))

        # Draw bounding boxes around detected people
        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Encode the frame for streaming
        ret, jpeg = cv2.imencode('.jpg', frame)
        frame = jpeg.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(person_detection_thread(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    capture_thread = threading.Thread(target=video_capture_thread)
    detection_thread = threading.Thread(target=person_detection_thread)

    capture_thread.start()
    detection_thread.start()

    # Start the Flask app
    app.run(host="127.0.0.1", debug=True)
