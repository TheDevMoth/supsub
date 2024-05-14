from pymavlink import mavutil
import time
from runner import setup_connection
import cv2

def main():
    """Main function to execute the vehicle control."""
    master = setup_connection(ip="192.168.2.1", port=14555)

    cap = cv2.VideoCapture("URL")

    while(cap.isOpened()):
        ret, frame = cap.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

