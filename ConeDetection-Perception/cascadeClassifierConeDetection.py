# This program detects cones in the image provided by the Wisconsin Autonomous
# Perception Programming Challenge.
# 
# The program takes inputs of a trained cascade classifier for detecting the
# cones and a boolean determining whether or not to display the bounding box
# rectangles around the detected cones. Following is an example of the command
# line argument to run this program:
# 
#   "python3 cascadeClassifierConeDetection.py --coneCascade=cascade.xml
#     --displayDetectedConeRectangles=1"
# 
# The program will display the image with lines drawn through the detected
# cones. When this image pops up, you may click 's' to save the image or any
# other key to terminate the program.
#
# Author: Jack Ludwikoski
# Email: jludwikoski@wisc.edu

import argparse
import cv2 as cv
import copy as cp


originalConeImage = cv.imread('originalConeImage.png')


# The command line arguments are defined and parsed
parser = argparse.ArgumentParser()
parser.add_argument('--coneCascade', help='Path to trained cascade XML.',
                    default='cascade.xml')
parser.add_argument('--displayDetectedConeRectangles',
                    help="""Enter \'1\' to draw rectangles around detected
                    cones. Otherwise, enter 0.""")
args = parser.parse_args()

# The cascade classifier which will be used to detect the cones is loaded
coneCascadePath = args.coneCascade
coneCascade = cv.CascadeClassifier()
if not coneCascade.load(cv.samples.findFile(coneCascadePath)):
    print('--(!)Error loading cone cascade')
    exit(0)

# The input for displaying the detected cone rectangles is converted to a
# boolean
displayDetectedConeRectangles = args.displayDetectedConeRectangles == '1'


# The input image is converted into high-contrast grayscale and then all cones
# in the image are detected and rectangles are drawn if specified
originalGray = cv.cvtColor(originalConeImage, cv.COLOR_BGR2GRAY)
originalGrayHighContrast = cv.equalizeHist(originalGray)
coneBoundingBoxes = coneCascade.detectMultiScale(originalGrayHighContrast,
                                                  1.3, maxSize=(250,300))
output = cp.deepcopy(originalConeImage)
if displayDetectedConeRectangles:
    for (x,y,w,h) in coneBoundingBoxes:
        cv.rectangle(output,(x,y),(x+w,y+h),(0,255,0),3)


# The detected cone center points are grouped into the left and right halves of
# the screen
horizCenter = output.shape[1] / 2
leftHalfCenterPts = []
rightHalfCenterPts = []
for (x,y,w,h) in coneBoundingBoxes:
    center = (x + w//2, y + h//2)
    if center[0] < horizCenter:
        leftHalfCenterPts.append(center)
    else:
        rightHalfCenterPts.append(center)

# This function takes an input image and a list of points as inputs.
# The function returns the input image with the linear regression line of the
# provided points drawn in red
def linearReg(image, points):
    a = 0
    b = 0
    xSum = 0
    ySum = 0
    for (x, y) in points:
        xSum += x
        ySum += y
    xAvg = xSum / len(points)
    yAvg = ySum / len(points)
    num = 0
    den = 0
    for (x, y) in points:
        num += (x - xAvg) * (y - yAvg)
        den += (x - xAvg)**2
    a = num/den
    b = yAvg - a * xAvg
    y1 = 0
    y2 = output.shape[0]
    x1 = int((y1 - b) // a)
    x2 = int((y2 - b) // a)
    cv.line(image,(x1,y1),(x2,y2),(0,0,255),6)
    return image

# The linear regression lines are drawn through the cones on each half of the
# screen
output = linearReg(output, leftHalfCenterPts)
output = linearReg(output, rightHalfCenterPts)


cv.imshow('Cone detection', output)

# The file is saved if the 's' key is pressed
k = cv.waitKey(0)
if k == ord('s'):
    cv.imwrite('answer.png', output)


