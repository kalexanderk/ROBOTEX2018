import cv2
import numpy as np
import copy
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
r = [len(frame)-200,len(frame)-190,0,len(frame[0])]
frame = cv2.resize(frame, (0,0), fx=0.4, fy=0.4)
detectcontrl = copy.deepcopy(frame)
#detectcontrl = detectcontrl[r[0]:r[1],r[2]:r[3]]

lH = 29
lS = 50
lV = 114
hH = 62
hS = 124
hV = 201


bil = 0
dil = 14
err = 0
ByAreaBool = 1

minAreaf = 600
maxAreaf = 100000
ByColorBool = 1
blobColorf = 255
ByCircularityBool = 1 
minCircularityf = 53
maxCircularityf = 100
ByConvexityBool = 0 
minConvexityf = 0
maxConvexityf = 100
ByInertiaBool = 0
minInertiaRatio = 0
maxInertiaRatio = 100


pBAB = ByAreaBool
pmA = minAreaf
pMA = maxAreaf
pBCB = ByColorBool
pbC = blobColorf
pBCB = ByCircularityBool 
pmCf = minCircularityf
pMCf = maxCircularityf
pbCB = ByConvexityBool 
pmCof = minConvexityf
pMCof = maxConvexityf
pbIB = ByInertiaBool
pmIr = minInertiaRatio
pMIr = maxInertiaRatio

def updateBABool(lw):
    global ByAreaBool
    ByAreaBool = lw
    return
def updateMiAF(lw):
    global minAreaf
    minAreaf = lw
    return
def updateMaAF(lw):
    global maxAreaf
    maxAreaf = lw
    return
def updateBCoBool(lw):
    global ByColorBool
    ByColorBool = lw
    return
def updateBCF(lw):
    global blobColorf
    blobColorf = lw
    return
def updateBCiBool(lw):
    global ByCircularityBool
    ByCircularityBool = lw
    return
def updateMiCF(lw):
    global minCircularityf
    minCircularityf = lw
    return
def updateMaCF(lw):
    global maxCircularityf
    maxCircularityf = lw
    return
def updateBConBool(lw):
    global ByConvexityBool
    ByConvexityBool = lw
    return
def updateMiConF(lw):
    global minConvexityf
    minConvexityf = lw
    return
def updateMaConF(lw):
    global maxConvexityf
    maxConvexityf = lw
    return
def updateBIBool(lw):
    global ByInertiaBool
    ByInertiaBool = lw
    return
def updateMiIR(lw):
    global minInertiaRatio
    minInertiaRatio = lw
    return
def updateMaIR(lw):
    global maxInertiaRatio
    maxInertiaRatio = lw
    return

def updatelH(lw):
    
    global lH
    lH = lw
    return
def updatelS(lw):
    
    global lS
    lS = lw
    return
def updatelV(lw):
    
    global lV
    lV = lw
    return
def updatehH(lw):
    
    global hH
    hH = lw
    return
def updatehS(lw):
    
    global hS
    hS = lw
    return
def updatehV(lw):
    
    global hV
    hV = lw
    return
def updatedil(lw):
    
    global dil
    dil = lw
    return
def updateErr(lw):
     
     global err
     err = lw
     return
def updateBil(lw):
     
     global bil
     bil = lw
     return
lowerLimits = np.array([lH, lS, lV])
upperLimits = np.array([hH, hS, hV])
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
thresholded = cv2.inRange(hsv, lowerLimits, upperLimits)
outimage = cv2.bitwise_and(hsv, hsv, mask = thresholded)
cv2.imshow('Threshold', outimage)
cv2.imshow('original', frame)
cv2.imshow('detectcontrl', detectcontrl)
cv2.createTrackbar("lH", "original", lH, 255, updatelH) 
cv2.createTrackbar("lS", "original", lS, 255, updatelS) 
cv2.createTrackbar("lV", "original", lV, 255, updatelV) 
cv2.createTrackbar("hH", "original", hH, 255, updatehH) 
cv2.createTrackbar("hS", "original", hS, 255, updatehS) 
cv2.createTrackbar("hV", "original", hV, 255, updatehV)

cv2.createTrackbar("Dilation", 'original', dil, 255, updatedil) 
cv2.createTrackbar("Erosion", 'original', err, 255, updateErr) 
cv2.createTrackbar("Gauss", "original", bil, 255, updateBil) 
cv2.createTrackbar("ByAreaBool", 'detectcontrl', ByAreaBool, 1, updateBABool)
cv2.createTrackbar("minAreaf", 'detectcontrl', minAreaf,100000, updateMiAF)
cv2.createTrackbar("maxAreaf", 'detectcontrl', maxAreaf, 100000, updateMaAF)
cv2.createTrackbar("ByColorBool", 'detectcontrl', ByColorBool, 1, updateBCoBool) 
cv2.createTrackbar("blobColorf", 'detectcontrl', blobColorf, 255, updateBCF)
cv2.createTrackbar("ByCircularityBool", 'detectcontrl', ByCircularityBool, 1, updateBCiBool) 
cv2.createTrackbar("minCircularityf", 'detectcontrl', minCircularityf, 100, updateMiCF)
cv2.createTrackbar("maxCircularityf", 'detectcontrl', maxCircularityf, 100, updateMaCF)
cv2.createTrackbar("ByConvexityBool", 'detectcontrl', ByConvexityBool, 1, updateBConBool) 
cv2.createTrackbar("minConvexityf", 'detectcontrl', minConvexityf, 100, updateMiConF)
cv2.createTrackbar("maxConvexityf", 'detectcontrl', maxConvexityf, 100, updateMaConF)
cv2.createTrackbar("ByInertiaBool", 'detectcontrl', ByInertiaBool, 1, updateBIBool) 
cv2.createTrackbar("minInertiaRatio", 'detectcontrl', minInertiaRatio, 100, updateMiIR)
cv2.createTrackbar("maxInertiaRatio", 'detectcontrl', maxInertiaRatio, 100, updateMaIR)
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = ByAreaBool
params.minArea = minAreaf
params.maxArea = maxAreaf
            
params.filterByColor = ByColorBool
params.blobColor = blobColorf
params.filterByCircularity = ByCircularityBool
params.minCircularity = minCircularityf/100.0
params.maxCircularity = maxCircularityf/100.0
            
params.filterByConvexity = ByConvexityBool
params.minConvexity = minConvexityf/100.0
params.maxConvexity = maxConvexityf/100.0
            
params.filterByInertia = ByInertiaBool
params.minInertiaRatio = minInertiaRatio/100.0
params.maxInertiaRatio = maxInertiaRatio/100.0


detector = cv2.SimpleBlobDetector(params)
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if (pBAB != ByAreaBool or pmA != minAreaf or pMA != maxAreaf or
        pBCB != ByColorBool or pbC != blobColorf or pBCB != ByCircularityBool or
        pmCf != minCircularityf or pMCf != maxCircularityf or
        pbCB != ByConvexityBool or pmCof != minConvexityf or
        pMCof != maxConvexityf or pbIB != ByInertiaBool or
        pmIr != minInertiaRatio or pMIr != maxInertiaRatio):
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea = ByAreaBool
            params.minArea = minAreaf
            params.maxArea = maxAreaf
            
            params.filterByColor = ByColorBool
            params.blobColor = blobColorf
            params.filterByCircularity = ByCircularityBool
            params.minCircularity = minCircularityf/100.0
            params.maxCircularity = maxCircularityf/100.0
            
            params.filterByConvexity = ByConvexityBool
            params.minConvexity = minConvexityf/100.0
            params.maxConvexity = maxConvexityf/100.0
            
            params.filterByInertia = ByInertiaBool
            params.minInertiaRatio = minInertiaRatio/100.0
            params.maxInertiaRatio = maxInertiaRatio/100.0
         
            detector = cv2.SimpleBlobDetector(params)
    
    ret, frame = cap.read()
    frame = cv2.resize(frame, (0,0), fx=0.4, fy=0.4)
    
    lowerLimits = np.array([lH, lS, lV])
    upperLimits = np.array([hH, hS, hV])
    
    imgcopy = copy.deepcopy(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    thresholded = cv2.inRange(hsv, lowerLimits, upperLimits)
    '''
    thresholded = cv2.GaussianBlur(thresholded, (1 + (2*bil), 1 + (2*bil)), 0)
    erernel = np.ones((1+err,1+err),np.uint8)
    thresholded = cv2.erode(thresholded,erernel,iterations = 1)
    dilernel = np.ones((1+dil,1+dil),np.uint8)
    thresholded = cv2.dilate(thresholded,dilernel,iterations = 1)
    '''
    outimage = cv2.bitwise_and(hsv, hsv, mask = thresholded)
    keypoints = detector.detect(thresholded)
    imgcopy = cv2.drawKeypoints(imgcopy, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('Threshold', outimage)
    cv2.imshow('original', imgcopy)
    cv2.imshow('detectcontrl', detectcontrl)

    if cv2.waitKey(1) & 0xFF == ord('s'):
        print(str(keypoints[0].size))
    
cv2.destroyAllWindows()
cap.release()
