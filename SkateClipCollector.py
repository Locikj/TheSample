import cv2 as cv
import numpy as np
import moviepy.editor as mpy


def Play_Video(videoPath):
    # read video
    capture = cv.VideoCapture(videoPath) #BGR
    
    #play each frame
    while True:
        isTrue, frame = capture.read()
        frame = cv.resize(frame, None, fx = .25, fy = .25, interpolation= cv.INTER_AREA) #resize frame
        cv.imshow('video', frame) # plays video in window

        if cv.waitKey(20) & 0xFF==ord('d'): # wait 20 seconds or press d key
            break
    capture.release()
    cv.destroyAllWindows()

    #does not work with 30fps video, finds 0 non zero elements
def Search_For_Black_Screen(videoPath, clipLength):
    capture = cv.VideoCapture(videoPath)
    clipMarker = 0 # holds the time where interesting frame was found
    # look at each frame
    while True:
        isTrue, frame = capture.read()
        frame = cv.resize(frame, (500,500), interpolation= cv.INTER_AREA)
        cv.imshow('video', frame) # plays video in window

        #check for black screen
        grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        print("non zero elements ", np.sum(grayFrame == 0))
        if np.sum(grayFrame == 0) == 0:
            print("Black screen detected")
            clipMarker = capture.get(cv.CAP_PROP_POS_MSEC) / 1000 #save time of frame in seconds
            if clipMarker < clipLength:
                continue
            return clipMarker, clipLength


        if cv.waitKey(20) & 0xFF==ord('d'): # wait 20 seconds or press d key
            return 0
    capture.release()
    cv.destroyAllWindows()

# highlight objects in the video
def Search_For_Objects(videoPath):
    capture = cv.VideoCapture(videoPath)

    while True:
        isTrue, frame = capture.read()
        frame = cv.resize(frame, (300,300))
        grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        blank = np.zeros(frame.shape, dtype='uint8')

        #find contours
        blur = cv.blur(grayFrame, (7,7))
        canny = cv.Canny(blur,100 ,200)
        ret, thresh = cv.threshold(blur,150, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contouredImage = cv.drawContours(blank,contours, -1, (0,255,0), 3)
        cv.imshow('Object Search', grayFrame)
        cv.imshow('canny', canny)
        cv.imshow('contours', blank)

        #recognize a hand

        if cv.waitKey(20) & 0xFF==ord('d'):
            break

    capture.release()
    cv.destroyAllWindows()

#create clip with moviePy using clipMarker
def Create_Clip(videoPath, clipMarker, clipLength):
    fullVideo = mpy.VideoFileClip(videoPath)
    if clipMarker - clipLength >= 0:
        clip = fullVideo.subclip(clipMarker - clipLength, clipMarker)
        clip.write_videofile("Videos/testClip.mp4")
    else:
        print("clip too short")

def Resize(videoPath, height, width):
    pass
#Play_Video("Videos/testClip.mp4")
#marker, length = Search_For_Black_Screen("Videos/30fps.mp4", 2)
#Create_Clip("Videos/30fps.mp4", marker , length)
Search_For_Objects("Videos/60fps.mp4")
