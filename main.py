import RPi.GPIO as GPIO
import time
import threading
import cv2
import datetime
import os
import requests

# Motor pin definitions
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# Tracking sensor pin definitions
TrackSensorLeftPin1  =  3  
TrackSensorLeftPin2  =  5   
TrackSensorRightPin1 =  4   
TrackSensorRightPin2 =  18  

# Servo pin definition for physical pin 23 (WiringPi pin 13)
SERVO_PIN = 13  # WiringPi pin 13 (Physical pin 23)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up GPIO pins
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)

GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
GPIO.setup(TrackSensorRightPin1, GPIO.IN)
GPIO.setup(TrackSensorRightPin2, GPIO.IN)

GPIO.setup(SERVO_PIN, GPIO.OUT)  # Set up servo pin
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for the servo
servo_pwm.start(0)

pwm_ENA = GPIO.PWM(ENA, 2000)
pwm_ENB = GPIO.PWM(ENB, 2000)
pwm_ENA.start(0)
pwm_ENB.start(0)

# Reduced speed for slower driving
CarSpeedControl = 30  # Adjust this value as needed

recording = threading.Event()
is_on_track = False
video_writer = None

def set_servo_angle(angle):
    duty = 2 + (angle / 18)  # Calculate the duty cycle for the angle
    GPIO.output(SERVO_PIN, True)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Allow time for the servo to move
    GPIO.output(SERVO_PIN, False)
    servo_pwm.ChangeDutyCycle(0)  # Stop sending signal

def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

def back():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)

def brake():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def tracking_test():
    global is_on_track

    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

    state = (TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2)
    if state != (1,1,1,1):
        is_on_track = True
    if state == (0, 0, 0, 0):
        run()
    elif state == (1, 0, 0, 0):
        right()
    elif state == (0, 1, 0, 0):
        right()
    elif state == (0, 0, 1, 0):
        left()
    elif state == (0, 0, 0, 1):
        left()
    elif state == (1, 1, 0, 0):
        right()
    elif state == (0, 0, 1, 1):
        left()
    elif state == (0, 1, 1, 0):
        left()
    elif state == (1, 0, 0, 1):
        right()
    elif state == (0, 1, 0, 1):
        right()
    elif state == (1, 0, 1, 0):
        left()
    elif state == (0, 1, 1, 1):
        left()
    elif state == (1, 1, 0, 1):
        right()
    elif state == (1, 1, 1, 0):
        right()
    elif state == (1, 1, 1, 1):
        is_on_track = False
        brake()

def upload_video(file_path):
    """
    Uploads a video file to the specified endpoint with a bearer token.

    Args:
        file_path (str): The path to the video file to upload.
    """
    url = 'https://app-app-dev-eus2-004.azurewebsites.net/api/v1/videos/upload'
    token = 'YOUR_BEARER_TOKEN_HERE'  # Replace with your actual bearer token

    headers = {
        'Authorization': f'Bearer {token}',
        'Accept-Language': 'en-US',
    }

    if not os.path.isfile(file_path):
        print(f"File not found: {file_path}")
        return

    try:
        with open(file_path, 'rb') as f:
            files = {
                'file': (os.path.basename(file_path), f, 'video/mp4')  # Updated MIME type
            }
            response = requests.post(url, headers=headers, files=files)

        if response.status_code in [200, 201]:
            print(f"Successfully uploaded {file_path}")
        else:
            print(f"Failed to upload {file_path}. Status Code: {response.status_code}, Response: {response.text}")
    except Exception as e:
        print(f"An error occurred while uploading {file_path}: {e}")

def video_recording():
    print("Video recording thread started.")
    global video_writer
    filename = None

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video capture device.")
        return

    videos_dir = 'videos'
    if not os.path.exists(videos_dir):
        os.makedirs(videos_dir)

    while True:
        if recording.is_set():
            if video_writer is None:
                filename = os.path.join(videos_dir, datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.mp4')
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))

            ret, frame = cap.read()
            if ret:
                video_writer.write(frame)
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Error reading from camera.")
                break
        else:
            if video_writer is not None:
                print("Recording stopped.")
                video_writer.release()
                video_writer = None
                if filename:
                    upload_thread = threading.Thread(target=upload_video, args=(filename,))
                    upload_thread.start()
        time.sleep(0.1)

    cap.release()
    if video_writer is not None:
        video_writer.release()
        if filename:
            upload_thread = threading.Thread(target=upload_video, args=(filename,))
            upload_thread.start()
    cv2.destroyAllWindows()
    print("Video recording thread terminated.")

def robot_control():
    global is_on_track
    print("Robot control thread started.")
    last_pause_time = time.time()

    while True:
        tracking_test()
        current_time = time.time()

        # Servo rotation and recording when robot stops
        if current_time - last_pause_time >= 2:
            print("Pausing for 2 seconds...")
            brake()

            # Start recording before rotating the servo
            if not recording.is_set():
                print("Starting recording...")
                recording.set()

            # Rotate servo to 90 degrees
            print("Rotating servo to 90°...")
            set_servo_angle(90)
            time.sleep(1)

            # Rotate servo to -90 degrees
            print("Rotating servo to -90°...")
            set_servo_angle(-90)
            time.sleep(1)

            # Return servo to 0 degrees
            print("Returning servo to 0°...")
            set_servo_angle(0)
            time.sleep(1)

            # Stop recording after full rotation
            if recording.is_set():
                print("Stopping recording...")
                recording.clear()

            last_pause_time = time.time()

        if is_on_track:
            if not recording.is_set():
                print("Starting recording due to tracking...")
                recording.set()
        else:
            if recording.is_set():
                print("Stopping recording due to lost tracking...")
                recording.clear()

        time.sleep(0.05)

    print("Robot control thread terminated.")

if __name__ == "__main__":
    # Start video recording thread
    recording_thread = threading.Thread(target=video_recording, daemon=True)
    recording_thread.start()

    # Start robot control thread
    control_thread = threading.Thread(target=robot_control, daemon=True)
    control_thread.start()

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Interrupted by user. Shutting down...")
    finally:
        recording.clear()
        pwm_ENA.stop()
        pwm_ENB.stop()
        servo_pwm.stop()
        GPIO.cleanup()
        print("Cleanup done. Exiting.")
