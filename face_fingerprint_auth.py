import cv2
import os
import serial
from datetime import datetime
import numpy as np

# Paths for storing admin data
ADMIN_FACE_PATH = "admin_face.jpg"
ADMIN_FINGERPRINT_PATH = "admin_fingerprint.txt"
UART_PORT = "COM3"  # Replace with your fingerprint sensor COM port
BAUD_RATE = 57600

# Create serial connection
def init_fingerprint_sensor():
    try:
        ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        print(f"Serial port {UART_PORT} opened successfully.")
        return ser
    except Exception as e:
        print(f"Error initializing serial port: {e}")
        return None

# Capture fingerprint
def capture_fingerprint(ser):
    try:
        print("Capturing fingerprint...")
        handshake_command = b'\xEF\x01\xFF\xFF\xFF\xFF\x01\x00\x03\x01\x00\x05'
        ser.write(handshake_command)
        response = ser.read(12)
        if response:
            print(f"Fingerprint captured: {response.hex()}")
            return response.hex()
        else:
            print("No response from fingerprint sensor.")
            return None
    except Exception as e:
        print(f"Error capturing fingerprint: {e}")
        return None

# Capture face
def capture_face():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not access the camera.")
        return None

    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    print("Press 'c' to capture face, 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cv2.imshow("Face Capture", frame)

        key = cv2.waitKey(1)
        if key == ord('c') and len(faces) > 0:
            print("Face captured.")
            face_image = frame[y:y+h, x:x+w]
            cap.release()
            cv2.destroyAllWindows()
            return face_image
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return None

# Enroll admin
def enroll_admin(ser):
    print("Enrolling admin...")
    face = capture_face()
    if face is not None:
        cv2.imwrite(ADMIN_FACE_PATH, face)
        print(f"Admin face saved at {ADMIN_FACE_PATH}")

    fingerprint = capture_fingerprint(ser)
    if fingerprint:
        with open(ADMIN_FINGERPRINT_PATH, "w") as f:
            f.write(fingerprint)
        print(f"Admin fingerprint saved at {ADMIN_FINGERPRINT_PATH}")

# Recognize admin face
def recognize_face(face_image):
    if not os.path.exists(ADMIN_FACE_PATH):
        print("Admin face data not found.")
        return False

    admin_face = cv2.imread(ADMIN_FACE_PATH, cv2.IMREAD_COLOR)
    face_image = cv2.resize(face_image, (admin_face.shape[1], admin_face.shape[0]))
    difference = cv2.absdiff(admin_face, face_image)
    if np.mean(difference) < 50:  # Adjust threshold if needed
        return True
    return False

# Main logic
def main():
    ser = init_fingerprint_sensor()
    if not ser:
        print("Fingerprint sensor initialization failed. Exiting.")
        return

    if not os.path.exists(ADMIN_FACE_PATH) or not os.path.exists(ADMIN_FINGERPRINT_PATH):
        enroll_admin(ser)
    else:
        print("Admin data found. Checking identity...")
        face = capture_face()
        if face is not None and recognize_face(face):
            print("Welcome Admin All Controls are open!")
        else:
            print("Unknown user detected.")
            print("Options:")
            print("1. Open doors only")
            print("2. Ignition on")
            print("3. Deny")
            choice = input("Enter your choice (1/2/3): ")
            if choice == "1":
                print("Opening doors...")
            elif choice == "2":
                print("Ignition turned on...")
            elif choice == "3":
                print("Access denied.")
            else:
                print("Invalid choice.")

if __name__ == "__main__":
    main()
