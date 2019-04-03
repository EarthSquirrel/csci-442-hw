# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import maestro
import sys
import threading
from client import ClientSocket

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
# face_cascade = cv.CascadeClassifier('lbpcascade_frontalface_improved.xml')

servo = maestro.Controller()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000


# Set everything to 0 to start
def stop():
    global motors, turn, body, headTurn, headTilt
    motors = 6000
    turn = 6000
    body = 6000
    headTurn = 6000
    headTilt = 6000

    servo.setTarget(MOTORS, motors)
    servo.setTarget(TURN, turn)
    servo.setTarget(HEADTURN, headTurn)
    servo.setTarget(HEADTILT, headTilt)
    servo.setTarget(BODY, body)


stop()

max_move = 5000  # 5400
max_turn = 7000  # left
min_turn = 5000  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5

tilt_positions = [5000, 6000, 7000]
tilt_loc = 0  # location in positions
increasing = True  # tell what direction it's going
eof = False  # Move every other frame
face_timer = max_time + 1
END_PROGRAM = False # use this to kill the threads
move_wait_time = 1.0  # time to wait before moving to new position head
frame_itter = 0
repositioning = False

def time_the_faces():
    if END_PROGRAM:
        return
    global face_timer
    face_timer += 1
    print('\t\tface_timer: ', face_timer)
    threading.Timer(1,time_the_faces).start()

def search():
    # print('searching.....')
    global increasing, headTurn, headTilt, tilt_loc
    if END_PROGRAM:
        stop()
        return
    if increasing:
        headTurn += 200
        if headTurn > max_head_turn:
            headTurn = max_head_turn
            increasing = False
    else:
        headTurn -= 200
        if headTurn < min_head_turn:
            headTurn = min_head_turn
            increasing = True
            tilt_loc += 1
            if tilt_loc > 2:
                # maxed out array, return to o
                tilt_loc= 0
            headTilt = tilt_positions[tilt_loc]
    print("Head Pos: ", headTurn)
    servo.setTarget(HEADTURN, headTurn)
    servo.setTarget(HEADTILT, headTilt)

    #if not face_found:
        #threading.Timer(move_wait_time, search).start()


threading.Timer(1, time_the_faces).start()
face_found = False
chase_human = False
search()
head_pos = servo.getPosition(HEADTURN)

def talk():
    IP = '10.200.22.237'
    # IP = sys.argv[1]
    PORT = 5010
    client = ClientSocket(IP, PORT)
    ##client.start()

    for i in ["hello human", "How are you", "Sorry, you must die now"]:
        time.sleep(1)
        client.sendData(i)
    print("Exiting Sends")

def reposition(turn_dir, head_pos, frame):
    global turn
    move_dist = abs(6000 - head_pos)/6000 * 1.5
    print('this mean move distance is.... ', move_dist)
    if turn_dir == 'left':
        print('reposition to the left')
        # turn -= 200
        turn = min_turn
        if turn < min_turn:
            turn = min_turn
    elif turn_dir == 'right':
        print('reposition to the right')
        # turn += 200
        turn = max_turn
        if turn > max_turn:
            turn = max_turn
    print('\t' + str(servo.getPosition(TURN)))
    servo.setTarget(TURN, turn)
    time.sleep(0.25)
    turn = 6000
    servo.setTarget(TURN, turn)
    time.sleep(.5)

def go_forward():
    servo.setTarget(MOTORS, 5200)

def faces_found(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1)
    servo.setTarget(TURN, turn)
    real_faces = []
    # if len(faces) > 0:
        # print('Faces total found: {}'.format(len(faces)))
    for (x,y,w,h) in faces:
        roi_gray = gray[y:y+h, x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        # If there are no eyes1111, don't use this face
        if len(eyes) == 0:
            continue
        #Otherwise, we use it and return it to our list of return faces
        # print('total eye count: {}'.format(len(eyes)))
        real_faces.append((x,y,w,h))
        # cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    return real_faces

# allow the camera to warmup
time.sleep(0.1)
turn_dir = "boom"
try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame_itter += 1
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        width, height, channel = image.shape

        faces = faces_found(image)
        if chase_human:
            print("Chase the human!!!!")
            face = faces[0]
            if face[2] > width * .5:
                stop()
            else:
                go_forward()
        elif repositioning:
            print("repositioning...")
            face_found = True
            face_timer = 0
            reposition(turn_dir, head_pos, frame)
            if len(faces) > 0:
                print('found a face!')
                face_center = faces[0][0] + 0.5*faces[0][2]
                if abs(face_center - width/2) < width/5:
                    print('Face in center, stoping things')
                    servo.setTarget(TURN, 6000)
                    repositioning = False
                    chase_human = True
        # elif len(faces) > 0 and not chase_human:
        # the first time the face found
        elif len(faces) > 0 and not face_found:
            print('greet the human!!!!')
            repositioning = True
            face_found = True
            # talk()
            head_pos = servo.getPosition(HEADTURN)
            if head_pos < 6000:
                print("Turn dir = left")
                turn_dir = 'left'
            elif head_pos:
                print("Turn dir = right")
                turn_dir = "right"
            servo.setTarget(HEADTURN, 6000)
            servo.setTarget(HEADTILT, 6000)
            reposition(turn_dir, head_pos, frame)
            face_timer = 0
            # cv.imwrite('frame' + str(frame_itter) + '.png', image)
            # TODO: Start chasing the human (find location of human)

        # The face has been found before
        elif len(faces) > 0:
            face_found = True
            face_timer = 0
            print('found a face!')
        # elif len(faces) > 0 and not chase_human:
            # elif face_found and not chase_human:

            # print('should be in location to chase the human down!!!!')
            # reposition(turn_dir, head_pos, frame)
            # check if face in correct location on screen?
            # if yes, chase human true and repositioning false?
            # x y w h

        # lost the face, but still not for long enough
        # elif face_timer < max_time:
        #    face_found = True

        # The timer is up!
        # No longer chacing human from here down
        elif face_found:
            # The time has run out and there are no faces
            face_found = False
            # assume the robot is chacing the human or doing something, stop it
            stop()
            chase_human = False
            # threading.Timer(move_wait_time, search).start()
        else:
            # do nothing here, keeps going as normal, no face
            # ..........
            face_found = False
            chase_human = False
            threading.Timer(move_wait_time, search).start()
            # threading.Timer(move_wait_time, search).start()

        # draw rectangle around the face
        for (x,y,w,h) in faces:
            cv.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

        cv.imshow("Frame", image)
        key = cv.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break

except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    END_PROGRAM = True
    stop()
    print('Stopped motors due to an error')
    face_timer = -10
