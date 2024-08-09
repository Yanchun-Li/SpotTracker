# import cv2
# camera = "/base/axi/pcie@120000/rp1/i2c@80000/ov5647@36"
# pipeline = "libcamerasrc camera-name=%s ! video/x-raw,width=640,height=480,framerate=10/1,format=RGBx ! videoconvert ! videoscale ! video/x-raw,width=640,height=480,format=BGR ! appsink" % (camera)
# cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

from picamera2 import Picamera2
import cv2

def main():
    # Initialize the camera
    picam2 = Picamera2()

    # Configure the camera to preview mode
    preview_config = picam2.create_preview_configuration()
    picam2.configure(preview_config)

    # Start the camera
    picam2.start()

    # Capture and display the video frames
    while True:
        frame = picam2.capture_array()
        cv2.imshow("Camera Feed", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    picam2.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
