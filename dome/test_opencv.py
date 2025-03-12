#!/usr/bin/env python3
# Pito Salas
import cv2
import sys

def main():
    # Open the default camera (usually camera index 0)
    # If you have multiple cameras, you might need to change the index
    print("Opening camera...")
    cap = cv2.VideoCapture(0)
    
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)
    
    print("Camera opened successfully. Press 'q' to quit.")
    
    # Continuously capture frames
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # If frame is not read correctly, break the loop
        if not ret:
            print("Error: Can't receive frame (stream end?). Exiting...")
            break
        
        # Display the resulting frame
        cv2.imshow('Camera Feed', frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released and program ended.")

if __name__ == "__main__":
    main()