# All packages needed for the program are imported ahead
import cv2
import numpy as np
import pyautogui
import time

# Some global variables or others that need prior intialization are initalized here

# colour ranges for feeding to the inRange funtions
blue_range = np.array([[88,78,20],[128,255,255]])
yellow_range = np.array([[18,103,71],[61,255,255]])
red_range = np.array([[36,39,16],[180 ,255,255]])


# Prior initialization of all centers for safety
y_cen, b_pos, r_cen = [240,320],[240,320],[240,320]
cursor = [960,540]
(sx,sy)=pyautogui.size()
(camx,camy)=320,240
pyautogui.FAILSAFE=False

# Area ranges for contours of different colours to be detected
r_area = [100,1700]
b_area = [100,1700]
y_area = [100,1700]

# Rectangular kernal for eroding and dilating the mask for primary noise removal
kernel = np.ones((7,7),np.uint8)

# Status variables defined globally
perform = False
showCentroid = False

# 'nothing' function is useful when creating trackbars
# It is passed as last arguement in the cv2.createTrackbar() function
def nothing(x):
    pass

# To bring to the top the contours with largest area in the specified range
# Used in drawContour()
def swap( array, i, j):
    temp = array[i]
    array[i] = array[j]
    array[j] = temp

# Distance between two centroids
def distance( c1, c2):
    distance = pow( pow(c1[0]-c2[0],2) + pow(c1[1]-c2[1],2) , 0.5)
    return distance

# To toggle status of control variables
def changeStatus(key):
    global perform
    global showCentroid
    global yellow_range,red_range,blue_range
    # toggle mouse simulation
    if key == ord('p'):
        perform = not perform
        if perform:
            print ('Mouse simulation ON...')
        else:
            print ('Mouse simulation OFF...')

    # toggle display of centroids
    elif key == ord('c'):
        showCentroid = not showCentroid
        if showCentroid:
            print ('Showing Centroids...')
        else:
            print ('Not Showing Centroids...')

    elif key == ord('r'):
        print ('**********************************************************************')
        print ('    You have entered recalibration mode.')
        print (' Use the trackbars to calibrate and press SPACE when done.')
        print ('    Press D to use the default settings')
        print ('**********************************************************************')

        yellow_range = calibrateColor('Yellow', yellow_range)
        red_range = calibrateColor('Green', red_range)
        blue_range = calibrateColor('Blue', blue_range)

    else:
        pass

# cv2.inRange function is used to filter out a particular color from the frame
# The result then undergoes morphosis i.e. erosion and dilation
# Resultant frame is returned as mask
def makeMask(hsv_frame, color_Range):

    mask = cv2.inRange( hsv_frame, color_Range[0], color_Range[1])
    # Morphosis next ...
    eroded = cv2.erode( mask, kernel, iterations=1)
    dilated = cv2.dilate( eroded, kernel, iterations=1)

    return dilated

# Contours on the mask are detected.. Only those lying in the previously set area
# range are filtered out and the centroid of the largest of these is drawn and returned
def drawCentroid(vid,color_name,  color_area, mask, showCentroid):
    global area

    _, contour, _ = cv2.findContours( mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    l=len(contour)
    area = np.zeros(l)

    # filtering contours on the basis of area rane specified globally
    for i in range(l):
        if cv2.contourArea(contour[i])>color_area[0] and cv2.contourArea(contour[i])<color_area[1]:
            area[i] = cv2.contourArea(contour[i])
        else:
            area[i] = 0

    a = sorted( area, reverse=True)

    # bringing contours with largest valid area to the top
    for i in range(l):
        for j in range(1):
            if area[i] == a[j]:
                swap( contour, i, j)

    if l > 0 :
        # finding centroid using method of 'moments'
        M = cv2.moments(contour[0])
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            center = (cx,cy)
            if showCentroid:

                if color_name == 'Green':
                    col= (0,255,0)
                elif color_name == 'Blue':
                    col = (255,0,0 )
                elif color_name == 'Yellow':
                    col = (8,255 ,235 )
                else:
                    print('Default')
                    col = (180, 255, 255)

                cv2.circle( vid, center, 5, col, -1)


            return center
    else:
        # return error handling values
        return (-1,-1)

# This function helps in filtering the required colored objects from the background
def calibrateColor(color, def_range):

    global kernel
    name = 'Calibrate '+ color
    cv2.namedWindow(name)
    cv2.createTrackbar('Hue', name, def_range[0][0]+20, 180, nothing)
    cv2.createTrackbar('Sat', name, def_range[0][1]   , 255, nothing)
    cv2.createTrackbar('Val', name, def_range[0][2]   , 255, nothing)
    while True:
        ret , frameinv = cap.read()
        frame=cv2.flip(frameinv ,1)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hue = cv2.getTrackbarPos('Hue', name)
        sat = cv2.getTrackbarPos('Sat', name)
        val = cv2.getTrackbarPos('Val', name)

        lower = np.array([hue-20,sat,val])
        upper = np.array([hue+20,255,255])

        mask = cv2.inRange(hsv, lower, upper)
        eroded = cv2.erode( mask, kernel, iterations=1)
        dilated = cv2.dilate( eroded, kernel, iterations=1)
        out = cv2.bitwise_and(frame,frame, mask =dilated)

        cv2.imshow(name, out)

        k = cv2.waitKey(3) & 0xFF
        if k == ord(' '):
            cv2.destroyWindow(name)
            return np.array([[hue-20,sat,val],[hue+20,255,255]])
        elif k == ord('d'):
            cv2.destroyWindow(name)
            return def_range

'''
This function takes as input the center of yellow region (yc) and
the previous cursor position (pyp). The new cursor position is calculated
in such a way that the mean deviation for desired steady state is reduced.
'''
def setCursorPos( yc, pyp):
    global sx
    global sy
    yp = np.zeros(2)

    if abs(yc[0] - pyp[0]) < 5 and abs(yc[1] - pyp[1]) < 5:
        yp[0] = yc[0] + .5 * (pyp[0] - yc[0])
        yp[1] = yc[1] + .5 * (pyp[1] - yc[1])
    else:
        yp[0] = yc[0] + .1 * (pyp[0] - yc[0])
        yp[1] = yc[1] + .1 * (pyp[1] - yc[1])

    return yp


# Depending upon the relative positions of the three centroids, this function chooses whether
# the user desires free movement of cursor, left click, right click or dragging
def chooseAction(bp, rc, yc):
    out = np.array(['move', 'false'])
    if rc[0]!=-1 and yc[0]!=-1:

        if distance(bp,rc)<50 and distance(bp,yc)<50 and distance(rc,yc)<50 :
            out[0] = 'drag'
            out[1] = 'true'
            return out
        elif distance(rc,yc)<40:#g+y =left  thumb +middle
            out[0] = 'left'
            return out
        elif distance(bp,rc)<40:#b+r index +thumb
            out[0] = 'right'
            return out
        elif distance(bp,rc)>40 and rc[1]-bp[1]>120:#index   down
            out[0] = 'down'
            return out
        elif bp[1]-rc[1]>110:
            out[0] = 'up'
            return out
        else:
            return out

    else:
        out[0] = -1
        return out

# Movement of cursor on screen, left click, right click,scroll up, scroll down
# and dragging actions are performed here based on value stored in 'action'.
def performAction( bp, rc, yc, action, drag, perform):
    global cursor
    global b_pos
    global sx
    global sy
    (camx, camy) = 640, 480
    if perform:
         cursor[0] = 4*(bp[0]-110)
         cursor[1] = 4*(bp[1]-120)
         if action == 'move':
            print ('Moving ')
            print (cursor[0], cursor[1])
            if bp[0]>110 and bp[0]<590 and bp[1]>120 and bp[1]<390:
                pyautogui.moveTo(cursor[0],cursor[1],duration = 0.2)
            elif bp[0]<110 and bp[1]>120 and bp[1]<390:
                pyautogui.moveTo( 8 , cursor[1],duration = 0.2)
            elif bp[0]>590 and bp[1]>120 and bp[1]<390:
                pyautogui.moveTo(1912, cursor[1],duration = 0.2)
            elif bp[0]>110 and bp[0]<590 and bp[1]<120:
                pyautogui.moveTo(cursor[0] , 8,duration = 0.2)
            elif bp[0]>110 and bp[0]<590 and bp[1]>390:
                pyautogui.moveTo(cursor[0] , 1072,duration = 0.2)
            elif bp[0]<110 and bp[1]<120:
                pyautogui.moveTo(8, 8,duration = 0.2)
            elif bp[0]<110 and bp[1]>390:
                pyautogui.moveTo(8, 1072,duration = 0.2)
            elif bp[0]>590 and bp[1]>390:
                pyautogui.moveTo(1912, 1072,duration = 0.2)
            else:
                pyautogui.moveTo(1912, 8,duration = 0.2)
         elif action == 'left':
            pyautogui.click(button = 'left')
            time.sleep(0.3)
         elif action == 'right':
            pyautogui.click(button = 'right')
            time.sleep(0.3)

         elif action == 'up':
            pyautogui.scroll(5)
#            time.sleep(0.3)

         elif action == 'down':
            pyautogui.scroll(-5)
#            time.sleep(0.3)

         elif action == 'drag' and drag == 'true':
            global b_pos
            drag = 'false'
            pyautogui.mouseDown()

            while True:

                k = cv2.waitKey(3) & 0xFF
                changeStatus(k)

                _, frameinv = cap.read()
                # flip horizontaly to get mirror image in camera
                frame = cv2.flip( frameinv, 1)

                hsv = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV)

                b_mask = makeMask( hsv, blue_range)
                r_mask = makeMask( hsv, red_range)
                y_mask = makeMask( hsv, yellow_range)

                py_pos = b_pos

                b_cen = drawCentroid(frame, 'Blue', b_area, b_mask, showCentroid)
                r_cen = drawCentroid(frame, 'Green', r_area, r_mask, showCentroid)
                y_cen = drawCentroid(frame, 'Yellow', y_area, y_mask, showCentroid)

                if     py_pos[0]!=-1 and b_cen[0]!=-1:
                    b_pos = setCursorPos(b_cen, py_pos)

                performAction(b_pos, r_cen, y_cen, 'move', drag, perform)
                cv2.imshow('Frame', frame)

                if distance(b_pos,r_cen)>60 or distance(b_pos,y_cen)>60 or distance(r_cen,y_cen)>60:
                    break

            pyautogui.mouseUp()



cap = cv2.VideoCapture(0)
def main():
    global yellow_range
    global red_range
    global blue_range
    global showCentroid
    global perform
    global b_pos

    print ('**********************************************************************')
    print ('    You have entered calibration mode.')
    print (' Use the trackbars to calibrate and press SPACE when done.')
    print ('    Press D to use the default settings.')
    print ('**********************************************************************')

    yellow_range = calibrateColor('Yellow', yellow_range)
    red_range = calibrateColor('Green', red_range)
    blue_range = calibrateColor('Blue', blue_range)
    print ('    Calibration Successfull...')

    cv2.namedWindow('Frame')

    print ( '**********************************************************************')
    print ('    Press P to turn ON and OFF mouse simulation.')
    print ('    Press C to display the centroid of various colours.')
    print ('    Press R to recalibrate color ranges.')
    print ('    Press ESC to exit.')
    print ('**********************************************************************')

    while True:

        k = cv2.waitKey(3) & 0xFF
        changeStatus(k)


        _, frameinv = cap.read()
        # flip horizontaly to get mirror image in camera
        frame = cv2.flip( frameinv, 1)

        hsv = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV)

        b_mask = makeMask( hsv, blue_range)
        r_mask = makeMask( hsv, red_range)
        y_mask = makeMask( hsv, yellow_range)

        py_pos = b_pos

        b_cen = drawCentroid(frame, 'Blue', b_area, b_mask, showCentroid)
        r_cen = drawCentroid(frame, 'Green', r_area, r_mask, showCentroid)
        y_cen = drawCentroid(frame, 'Yellow', y_area, y_mask, showCentroid)

        if     py_pos[0]!=-1 and b_cen[0]!=-1 and b_pos[0]!=-1:
            b_pos=setCursorPos(b_cen, py_pos)

        output = chooseAction(b_pos, r_cen, b_cen)
        if output[0]!=-1:
            performAction(b_pos, r_cen, y_cen, output[0], output[1], perform)

        cv2.imshow('Frame', frame)

        if k == 27:
            print ('exit')
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
