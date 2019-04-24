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

max_move = 5200  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5

# In the hall
max_move = 5600
#max_turn = 6800
#min_turn = 5000



tilt_positions = [5000, 6000, 7000]
tilt_loc = 0  # location in positions
increasing = True  # tell what direction it's going
eof = False  # Move every other frame
face_timer = max_time + 1
END_PROGRAM = False # use this to kill the threads
move_wait_time = 1.0  # time to wait before moving to new position head
frame_itter = 0
repositioning = False
searching = True
reposition_timer = 0

def time_the_faces():
    if END_PROGRAM:
        return
    global face_timer
    face_timer += 1
    print('\t\tface_timer: ', face_timer)
    threading.Timer(1,time_the_faces).start()

def time_reposition():
    if END_PROGRAM:
        return
    global reposition_timer
    reposition_timer += 1
    print('\t\treposition_timer: ', reposition_timer)
    if repositioning:
        threading.Timer(1,time_reposition).start()

def search():
    headTurn, headTilt, tilt_loc, searching
    increasing = False
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
    # print("\t\t\tSearching: Head Pos: ", headTurn)
    servo.setTarget(HEADTURN, headTurn)
    servo.setTarget(HEADTILT, headTilt)

    if not face_found:
        threading.Timer(move_wait_time, search).start()
    else: # no longer searching if found a face
        searching = False


threading.Timer(1, time_the_faces).start()
face_found = False
chase_human = False
stopped = False
search()
head_pos = servo.getPosition(HEADTURN)

def talk():
    IP = '10.200.2.215'
    PORT = 5010
    client = ClientSocket(IP, PORT)
    ##client.start()
    for i in ["hello human", "How are you", "Sorry, you must die now"]:
        client.sendData(i)
        print('\tspeaking: ', i)
        time.sleep(1)
    print("\tExiting Sends")
    client.killSocket()

def reposition(turn_dir, head_pos, frame):
    global turn
    if turn_dir == 'left':
        print('\treposition to the left')
        # turn -= 200
        turn = min_turn
        if turn < min_turn:
            turn = min_turn
    elif turn_dir == 'right':
        print('\treposition to the right')
        # turn += 200
        turn = max_turn
        if turn > max_turn:
            turn = max_turn
    else:
        print('Going streight')
        # print('\tYa, something broke....')
    servo.setTarget(TURN, turn)
    time.sleep(0.25)
    # print('\trepositioning turn value: ' + str(servo.getPosition(TURN)))
    turn = 6000
    servo.setTarget(TURN, turn)
    time.sleep(.5)

def go_forward():
    servo.setTarget(MOTORS, max_move)

def faces_found(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, minSize=(10,10), maxSize=(width, height))
    servo.setTarget(TURN, turn)
    real_faces = []
    # if len(faces) > 0:
        # print('Faces total found: {}'.format(len(faces)))
    for (x,y,w,h) in faces:
        roi_gray = gray[y:y+h, x:x+w]
        # don't worry about eyes if chacing human, need to get it!
        eyes = eye_cascade.detectMultiScale(roi_gray)
        # If there are no eyes1111, don't use this face
        if len(eyes) == 0:
            continue
        #Otherwise, we use it and return it to our list of return faces
        # print('total eye count: {}'.format(len(eyes)))
        real_faces.append((x,y,w,h))
        # cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    if len(real_faces) > 0:
        print('\tfaces_found has a face!')
    return real_faces

# allow the camera to warmup
time.sleep(0.1)
turn_dir = "boom"
# threading.Timer(.1, talk).start()
try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame_itter += 1
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        width, height, channel = image.shape
        if increasing: # moving robot left
            pass
        image = image[0:height, int(height/8):int(7*width/8)] # int(width*.25):int(width*.75)]
        faces = faces_found(image)
        # found human and sees no face
        if stopped and len(faces) == 0:
            # needs to wait seven seconds before searching again
            print('Lost the face, now I will wait 7 seconds...')
            time.sleep(7)
            stopped = False
            face_found = False
            repositioning = False
            chase_human = False
        # TODO: What happens with stop down here?
        elif chase_human:
            print("Chase the human!!!!")
            # The face has been lost too long, stop before people die
            if face_timer > 3 and not stopped:
                print('Was chacing, but lost face for too long. Stop!')
                print('*************DONE!*********************')
                stop()
                face_found = False
                chase_human = False
                repositioning = False
                searching = False
                stopped = True
            elif len(faces) > 0:
                face = faces[0]
                print('Face {} of screen'.format(str(face[2]/width*100)))
                # stop if width face is too much
                if stopped:
                    if face_timer > 7:
                        face_found = False
                        chase_human = False
                        repositioning = False
                        searching = False
                        stopped = False
                elif face[2]/width > .17:
                    print('Was chasing, but got a big face so stop!')
                    stop()
                    print('*************DONE!*********************')
                    stopped = True
                else:
                    go_forward()
            elif stopped:
                if face_timer > 7:
                    face_found = False
                    chase_human = False
                    repositioning = False
                    searching = True
                    stopped = False

            else:
                go_forward()

        elif repositioning:
            print("repositioning...")
            face_found = True
            reposition(turn_dir, head_pos, frame)
            # stop if face is too close
            if len(faces) > 0:
                face_timer = 0
                print('found a face!')
                face_center = faces[0][0] + 0.5*faces[0][2]
                if abs(face_center - width/2) < width/5:
                    print('Face in center, stoping things')
                    turn = 6000
                    servo.setTarget(TURN, turn)
                    repositioning = False
                    chase_human = True
            """
            elif reposition_timer > 6:
                print('Took to long to locate human, searching again!')
                stop()
                repositioning = False
                face_found = False
                searching = False
                chase_human = False
            """

        # elif len(faces) > 0 and not chase_human:

        # the first time the face found
        elif len(faces) > 0 and not face_found:
            print('greet the human!!!!')
            repositioning = True
            # reposition_timer = 0
            # threading.Timer(1, time_reposition).start()
            face_found = True
            threading.Timer(0, talk).start()
            head_pos = servo.getPosition(HEADTURN)
            face_loc = faces[0][0] + faces[0][2]/2
            # if (head_pos < 6000 and face_loc < width/2) or (head_pos > 6000 and face_loc < width/2):
            if head_pos > 6000:
                print("Turn dir = right")
                turn_dir = "right"
            # elif (head_pos > 6000 and face_loc > width/2) or (head_pos < 6000 and face_loc > width/2):
            if head_pos < 5000:
                print("Turn dir = left")
                turn_dir = 'left'
            headTilt, headTurn = 6000, 6000
            servo.setTarget(HEADTURN, headTurn)
            servo.setTarget(HEADTILT, headTilt)
            reposition(turn_dir, head_pos, frame)
            face_timer = 0
            # cv.imwrite('frame' + str(frame_itter) + '.png', image)

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

        # Searching when no face is found, not chacing or repositioning or
        # not already searching
        elif not face_found and not repositioning and not chase_human and not searching:
            searching = True
            threading.Timer(move_wait_time, search).start()

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
            # threading.Timer(move_wait_time, search).start()

        # TODO: Does this go in the big conditional or will it work out here?
        # I'm trying it here, because I'm lazy
        if face_found:
            searching = False
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
