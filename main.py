# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
import time
import threading
import cv2
import datetime

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
is_moving = False
video_writer = None

def run():
    global is_moving
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    is_moving = True

def back():
    global is_moving
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    is_moving = True

def left():
    global is_moving
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    is_moving = True

def right():
    global is_moving
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(CarSpeedControl)
    pwm_ENB.ChangeDutyCycle(CarSpeedControl)
    is_moving = True

def brake():
    global is_moving
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    is_moving = False

def tracking_test():
    global is_moving
    TrackSensorLeftValue1  = GPIO.input(TrackSensorLeftPin1)
    TrackSensorLeftValue2  = GPIO.input(TrackSensorLeftPin2)
    TrackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
    TrackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

    state = (TrackSensorLeftValue1, TrackSensorLeftValue2, TrackSensorRightValue1, TrackSensorRightValue2)
    
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
        brake()

def video_recording():
    global video_writer
    cap = cv2.VideoCapture(0)
    while True:
        if recording.is_set():
            if video_writer is None:
                # Create unique filename
                filename = datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.avi'
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writer = cv2.VideoWriter(filename, fourcc, 20.0, (640, 480))

            ret, frame = cap.read()
            if ret:
                video_writer.write(frame)
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break
        else:
            if video_writer is not None:
                video_writer.release()
                video_writer = None
        time.sleep(0.1)  # Sleep briefly to avoid high CPU usage

    cap.release()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()

def robot_control():
    global is_moving
    while True:
        tracking_test()
        if is_moving:
            if not recording.is_set():
                # Start recording after the robot has been moving for 0.5s
                time.sleep(0.5)
                recording.set()
        else:
            if recording.is_set():
                # Stop recording if the robot stops
                recording.clear()
        time.sleep(0.05)  # Shorter delay for finer control

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
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup()
