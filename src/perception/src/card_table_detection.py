# CARD DETECTOR.  MUCH OF CODE AND IDEAS TAKEN FROM
# https://www.pyimagesearch.com/2014/09/01/build-kick-ass-mobile-document-scanner-just-5-minutes/

import cv2
import numpy as np

#cap = cv2.VideoCapture("videos/head_cards2.avi")
def non_max_suppression_fast(boxes, overlapThresh):
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return []
    boxes = np.array(boxes)
    # if the bounding boxes integers, convert them to floats --
    # this is important since we'll be doing a bunch of divisions
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
 
    # initialize the list of picked indexes 
    pick = []
 
    # grab the coordinates of the bounding boxes
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]
 
    # compute the area of the bounding boxes and sort the bounding
    # boxes by the bottom-right y-coordinate of the bounding box
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)
 
    # keep looping while some indexes still remain in the indexes
    # list
    while len(idxs) > 0:
        # grab the last index in the indexes list and add the
        # index value to the list of picked indexes
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)
 
        # find the largest (x, y) coordinates for the start of
        # the bounding box and the smallest (x, y) coordinates
        # for the end of the bounding box
        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])
 
        # compute the width and height of the bounding box
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
 
        # compute the ratio of overlap
        overlap = (w * h) / area[idxs[:last]]
 
        # delete all indexes from the index list that have
        idxs = np.delete(idxs, np.concatenate(([last],
            np.where(overlap > overlapThresh)[0])))
 
    # return only the bounding boxes that were picked using the
    # integer data type
    return boxes[pick].astype("int")


def get_contours(frame, hand=True):
    frame = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hand:
        LOWERX, UPPERX, LOWERY, UPPERY = 100, 1100, 150, 800
    else:
        LOWERX, UPPERX, LOWERY, UPPERY = 300, 900, 0, 800

    gray = gray[LOWERY:UPPERY,LOWERX:UPPERX]
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 100, 150)
    cnts = cv2.findContours(edged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
    cv2.drawContours(frame, cnts, -1, (125, 0, 0), 3)
    cv2.imshow('img', edged)
    cv2.waitKey(1)
    print(len(cnts))
    return None, None, frame
    screenCnts = []
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        #(x,y),(w,h), ang = cv2.minAreaRect(c)
        x,y,w,h = cv2.boundingRect(c)
        boundingArea = w * h
        actualArea = cv2.contourArea(c)
        

        if len(approx) >= 4 and len(approx) <= 5 and actualArea > 0 \
                 and boundingArea > 10000 and boundingArea / actualArea < 1.5: #and len(approx) <=5:
            screenCnts += [[x,y, x+w, y+h]]
            cv2.putText(frame, "{}".format(boundingArea / actualArea),
                  (x+LOWERX, y+LOWERY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
    #cv2.drawContours(frame, screenCnts, -1, (0,255,0), 3)    
    print(len(screenCnts))
    screenCnts = non_max_suppression_fast(screenCnts, 0.3)
    blacks, whites = [],[]
    widths = []
    heights = []
    for pt in screenCnts:
        pt[0] += LOWERX
        pt[2] += LOWERX
        pt[1] += LOWERY
        pt[3] += LOWERY
        x1,y1,x2,y2 = pt
        widths.append(x2 - x1)
        heights.append(y2 - y1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_color = np.mean(gray[int(y1):int(y2), int(x1):int(x2)])
        cv2.putText(frame, "color: {}".format(int(mean_color)),
                 (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        if mean_color > 200:
            whites.append(pt)
        else:
            blacks.append(pt)

        cv2.rectangle(frame, (x1, y1), (x2, y2), 255, 2)
#    cv2.drawContours(frame, screenCnts, 0, 255)
    return blacks, whites, frame

if __name__ == "__main__":
    cap = cv2.VideoCapture("./videos/head_cards2.avi")
    i = 0
    while 1:
        ret, frame = cap.read()
        cv2.imwrite("./videos/test/img{}.png".format(i), frame)
        i+= 1
        continue

        # make image blurred grayscale and find the edges
        if frame is None: break
        _, _, frame = get_contours(frame, hand=False)
        #contours, frame_white = get_contours(frame, True, False)
        #contours, frame_black = get_contours(frame, True, True)
        #frame_white = cv2.resize(frame_white, (512, 300))
        #frame_black = cv2.resize(frame_white, (512, 300))
        #cv2.imshow("cards", np.hstack((frame_white, frame_black)))
        cv2.imshow("thres", frame)
        
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break

    cv2.destroyAllWindows()
    cap.release()
