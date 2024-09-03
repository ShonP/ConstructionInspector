# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
import threading
import cv2
import datetime
import os

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

pwm_ENA = GPIO.PWM(ENA, 2000)
pwm_ENB = GPIO.PWM(ENB, 2000)
pwm_ENA.start(0)
pwm_ENB.start(0)

# Reduced speed for slower driving
CarSpeedControl = 30  # Adjust this value as needed

recording = threading.Event()
is_on_track = False
video_writer = None

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
    elif state == (1, 0, 0, 1):
        right()
    elif state == (0, 1, 1, 1):
        left()
    elif state == (1, 1, 0, 1):
        right()
    elif state == (1, 1, 1, 0):
        right()
    elif state == (1, 1, 1, 1):
        is_on_track = False
        brake()
def video_recording():
    print("Video recording thread started.")
    global video_writer
    
    cap = cv2.VideoCapture(0)
    # Create the 'videos' directory if it doesn't exist
    videos_dir = 'videos'
    if not os.path.exists(videos_dir):
        os.makedirs(videos_dir)

    while True:
        if recording.is_set():
            if video_writer is None:
                filename = os.path.join(videos_dir, datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.avi')
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writer = cv2.VideoWriter(filename, fourcc, 5.0, (640, 480))
                print("Recording started: {}".format(filename))

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
        time.sleep(0.1)  # Sleep briefly to avoid high CPU usage

    cap.release()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()
    print("Video recording thread terminated.")
def robot_control():
    global is_on_track
    print("Robot control thread started.")
    last_pause_time = time.time()

    while True:
        tracking_test()
        current_time = time.time()
        if current_time - last_pause_time >= 2:
            print("Pausing for 2 seconds...")
            brake()
            time.sleep(2)  # Pause for 2 seconds
            last_pause_time = time.time()

        if is_on_track:
            if not recording.is_set():
                print("Starting recording...")
                time.sleep(0.5)
                recording.set()
        else:
            if recording.is_set():
                print("Stopping recording...")
                recording.clear()

        time.sleep(0.05)  # Shorter delay for finer control

    print("Robot control thread terminated.")

if __name__ == "__main__":
    # Start the video recording thread
    recording_thread = threading.Thread(target=video_recording)
    recording_thread.start()

    # Start the robot control thread
    control_thread = threading.Thread(target=robot_control)
    control_thread.start()

    try:
        control_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        recording.clear()
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup()
