# Library
import numpy as np
import cv2 as cv
#from Robot import robot
from circle import Circle

# Ball detection function
def findBall(im_RGB, colors = [True, True, True, True, True]):
    # Getting the image as HSV and grayscale
    im_HSV = cv.cvtColor(im_RGB, cv.COLOR_BGR2HSV)
    im_gray = cv.cvtColor(im_RGB, cv.COLOR_BGR2GRAY)

    # Preprocessing for ball detection
    im_gray = cv.medianBlur(im_gray, 5)
    im_HSV[:3 * im_HSV.shape[0] // 8] = 0
    im_gray[:3 * im_gray.shape[0] // 8] = 0

    # Mask for orange color (golf ball)
    if colors[0] == True:
        lowerBound = np.array([0, 90, 60])
        upperBound = np.array([15, 255, 255])
        orangeMask = cv.inRange(im_HSV, lowerBound, upperBound)
    else:
        orangeMask = np.zeros(im_gray.shape, np.uint8)

    # Mask for red color
    if colors[1] == True:
        lowerBound = np.array([155, 150, 40])
        upperBound = np.array([180, 255, 255])
        redMask1 = cv.inRange(im_HSV, lowerBound, upperBound)
        lowerBound = np.array([0, 150, 40])
        upperBound = np.array([15, 255, 255])
        redMask2 = cv.inRange(im_HSV, lowerBound, upperBound)
        redMask = cv.bitwise_or(redMask1, redMask2)
    else:
        redMask = np.zeros(im_gray.shape, np.uint8)
    
    # Mask for white color
    if colors[2] == True:
        lowerBound = np.array([0, 0, 100])
        upperBound = np.array([180, 70, 255])
        whiteMask = cv.inRange(im_HSV, lowerBound, upperBound)
    else:
        whiteMask = np.zeros(im_gray.shape, np.uint8)

    # Mask for blue color
    if colors[3] == True:
        lowerBound = np.array([95, 70, 90])
        upperBound = np.array([115, 255, 255])
        blueMask = cv.inRange(im_HSV, lowerBound, upperBound)
    else:
        blueMask = np.zeros(im_gray.shape, np.uint8)

    # Mask for brown color
    if colors[4] == True:
        lowerBound = np.array([5, 55, 0])
        upperBound = np.array([35, 255, 120])
        brownMask = cv.inRange(im_HSV, lowerBound, upperBound)
    else:
        brownMask = np.zeros(im_gray.shape, np.uint8)

    # Mask on the gray image (the hole)
    mask = cv.bitwise_or(orangeMask, redMask)
    mask = cv.bitwise_or(mask, whiteMask)
    mask = cv.bitwise_or(mask, blueMask)
    mask = cv.bitwise_or(mask, brownMask)
    im_gray = cv.bitwise_and(im_gray, im_gray, mask = mask)

    # Making the image binary
    kernel = np.ones((3, 3), np.uint8)
    im_canny = cv.Canny(im_gray, 50, 100)
    im_canny = cv.dilate(im_canny, kernel, iterations = 1)
    im_canny = cv.morphologyEx(im_canny, cv.MORPH_OPEN, kernel)

    # Finding the contours of the image
    contours, _ = cv.findContours(im_canny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    ellipse = []

    # Go through all the countours and find the ellipses
    for contour in contours:
        # Removing contours with too few points
        if len(contour) < 5:
            continue

        # Removing too small areas
        area = cv.contourArea(contour)
        
        #if (area < 8000 and colors[4] == False) or len(contour) < 5 or (area < 1000 and colors[4] == True):
        if (area < 300 and colors[4] == False) or len(contour) < 5 or (area < 80 and colors[4] == True):
            continue
        
        # Fitting contours to ellipses
        foundEllipse = cv.fitEllipse(contour)

        # Removing the ellipses outside the frame
        if (foundEllipse[0][0] > im_RGB.shape[1] or foundEllipse[0][0] < 0) or (foundEllipse[0][1] > im_RGB.shape[0] or foundEllipse[0][1] < 0):
            continue

        # Removing ellipses with too big ratio between axes
        ratio = foundEllipse[1][1] / foundEllipse[1][0] if foundEllipse[1][0] != 0 else float('inf')
        if (ratio > 2 and area < 800 and colors[4] == False) or (ratio > 1.5 and area >= 800 and colors[4] == False) or (ratio < 1.5 and colors[4] == True):
            continue
        
        # Removing overlapping ellipses
        if len(ellipse) > 1:
            if abs(foundEllipse[0][0] - ellipse[-1][0][0]) < 10 and abs(foundEllipse[0][1] - ellipse[-1][0][1]) < 10:
                # Check for biggest area
                if area > areaOld:
                    ellipse.pop()
                else:
                    continue
                continue

        # Appending accepted ellipses to the list
        ellipse.append(((np.round(foundEllipse[0][0]).astype(np.uint16), np.round(foundEllipse[0][1]).astype(np.uint16)),
                        (np.round(foundEllipse[1][0]).astype(np.uint16), np.round(foundEllipse[1][1]).astype(np.uint16)),
                         np.round(foundEllipse[2]).astype(np.uint16)))
        
        # Save old area
        areaOld = area

    # Finding out the color of each detected circles
    detectCircle = findColor(im_HSV, ellipse, colors)

    # Check if there are any detected circles
    if detectCircle.shape[0] == 0:
        print("No ball detected")
        return None

    # Returning the circles
    return detectCircle

# Find the color of the circles
def findColor(image, ellipses, colors = [True, True, True, True, True]):
    # Initialise the list
    detectColor = []
    for val in ellipses:
        # The HSV channels
        H_ch = image[val[0][1] - 1, val[0][0] - 1, 0]
        S_ch = image[val[0][1] - 1, val[0][0] - 1, 1]
        V_ch = image[val[0][1] - 1, val[0][0] - 1, 2]

        # Color argument with 0 for golf ball, 1 for red ball, 2 for white ball, 3 for blue ball and 4 for hole
        if (H_ch >= 0 and H_ch <= 15) and (S_ch >= 90 and S_ch <= 255) and (V_ch >= 60 and V_ch <= 255) and colors[0]:
            detectColor.append(np.array([val[0][0], val[0][1], val[1][0], val[1][1], val[2], 0]))
        elif (H_ch >= 155 or H_ch <= 15) and (S_ch >= 150 and S_ch <= 255) and (V_ch >= 40 and V_ch <= 255) and colors[1]:
            detectColor.append(np.array([val[0][0], val[0][1], val[1][0], val[1][1], val[2], 1]))
        elif (H_ch >= 0 and H_ch <= 180) and (S_ch >= 0 and S_ch <= 70) and (V_ch >= 100 and V_ch <= 255) and colors[2]:
            detectColor.append(np.array([val[0][0], val[0][1], val[1][0], val[1][1], val[2], 2]))
        elif (H_ch >= 95 and H_ch <= 115) and (S_ch >= 70 and S_ch <= 255) and (V_ch >= 90 and V_ch <= 255) and colors[3]:
            detectColor.append(np.array([val[0][0], val[0][1], val[1][0], val[1][1], val[2], 3]))
        elif (H_ch >= 15 and H_ch <= 30) and (S_ch >= 90 and S_ch <= 255) and (V_ch >= 0 and V_ch <= 150) and colors[4]:
            detectColor.append(np.array([val[0][0], val[0][1], val[1][0], val[1][1], val[2], 4]))
    
    # Convert to array
    detectColor = np.array(detectColor)

    # Return color information
    return detectColor

# Finding the 3D coordinate of the balls
def place3D(cameraCalMatrix, worldMat, distortion, ballClass):
    # The pixel, camera and world coordinates initialisation
    cameraPlace = np.ones((4, len(ballClass)))
    worldPlace = np.ones((4, len(ballClass)))

    # Remove distortion from points and transforming to camera coordinates (witout depth)
    points = np.array([ball.coordinate for ball in ballClass]).astype(np.float32).reshape(-1, 1, 2)
    undistortedPoints = cv.undistortPoints(points, cameraCalMatrix, distortion)

    # Looping through all the points
    for idx in range(len(ballClass)):
        # Depth information of the detected balls
        depth = cameraCalMatrix[0][0] * ballClass[idx].realDiameter / (2 * ballClass[idx].majorAxis)

        # Calculation of the camera coordinates (with depth)
        x_norm, y_norm = undistortedPoints[idx][0]
        cameraPlace[:-1, idx] = np.array([x_norm, y_norm, 1]) * depth
        
        # Calculation of the world coordinates
        worldPlace[:, idx] = np.dot(worldMat, cameraPlace[:, idx])
        worldPlace[:, idx] /= worldPlace[-1, idx]

    # Return world coordinates
    return worldPlace

# Calculating where the robot should move
def next_action(worldPlace, gripLength, turnRadius = 0.4):
    # Finding the closest ball
    lengthList = []
    for idx in range(0, worldPlace.shape[1]):
        length = np.sqrt(worldPlace[0, idx] ** 2 + worldPlace[1, idx] ** 2 + worldPlace[2, idx] ** 2)
        lengthList.append(length)
    lengthList = np.array(lengthList)
    closeIdx = np.argmin(lengthList)
    close = worldPlace[:, closeIdx]

    # Get the degrees to rotate, so that the ball is in center
    degree = -np.arctan2(close[1], close[0])
    
    # The robot turns instead of rotate, so the walk has to be accounted for
    rotate = np.array([[np.cos(degree), -np.sin(degree), 0, turnRadius * np.sin(degree)],
                       [np.sin(degree), np.cos(degree), 0, turnRadius * (1 - np.cos(degree))],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
    newWorldPlace = np.dot(np.linalg.inv(rotate), close)

    # Get the distance to move, so that the ball is 25 cm in front of the ball
    distance = newWorldPlace[0] - gripLength
    
    # Return the degree and distance
    return degree * 180 / np.pi, distance

# Remove extra balls found in the image
def remove_extra_ball(ball, amount = [1, 1, 3, 1, 1]):
    # Make a list of ball object
    ballClass = []
    for idx in range(0, ball.shape[0]):
        potentialBall = Circle(ball[idx])
        ballClass.append(potentialBall)

    ## Find all the type of balls
    allTypes = [obj.type for obj in ballClass]
    
    # Remove smallest golf balls
    while allTypes.count("G") > amount[0]:
        checkBall = [(idx, obj.coordinate[1]) for idx, obj in enumerate(ballClass) if obj.type == "G"]
        idx = np.argmin([row for _, row in checkBall])
        del ballClass[[index for index, _ in checkBall][idx]]
        allTypes = [obj.type for obj in ballClass]
    
    # Remove smallest red balls
    while allTypes.count("R") > amount[1]:
        checkBall = [(idx, obj.coordinate[1]) for idx, obj in enumerate(ballClass) if obj.type == "R"]
        idx = np.argmin([row for _, row in checkBall])
        del ballClass[[index for index, _ in checkBall][idx]]
        allTypes = [obj.type for obj in ballClass]
    
    # Remove smallest white balls
    while allTypes.count("W") > amount[2]:
        checkBall = [(idx, obj.coordinate[1]) for idx, obj in enumerate(ballClass) if obj.type == "W"]
        idx = np.argmin([row for _, row in checkBall])
        del ballClass[[index for index, _ in checkBall][idx]]
        allTypes = [obj.type for obj in ballClass]

    # Remove smallest blue balls
    while allTypes.count("B") > amount[3]:
        checkBall = [(idx, obj.coordinate[1]) for idx, obj in enumerate(ballClass) if obj.type == "B"]
        idx = np.argmin([row for _, row in checkBall])
        del ballClass[[index for index, _ in checkBall][idx]]
        allTypes = [obj.type for obj in ballClass]
    
    # Remove smallest holes
    while allTypes.count("H") > amount[4]:
        checkBall = [(idx, obj.coordinate[1]) for idx, obj in enumerate(ballClass) if obj.type == "H"]
        idx = np.argmin([row for _, row in checkBall])
        del ballClass[[index for index, _ in checkBall][idx]]
        allTypes = [obj.type for obj in ballClass]

    # Return the updated list of balls
    return ballClass

# Get information for the code in robocupmain.py
def get_ball_info(im_RGB, ballClass):
    # Getting the frame width
    frame_width = int(im_RGB.shape[1])
    
    # Getting the ball x center
    x_center = int(ballClass.coordinate[0])

    # Getting the area of the ball
    size = int(np.pi * ballClass.majorAxis * ballClass.minorAxis)

    # Return the frame_width, x center and size of the ball
    return frame_width, x_center, size

# Show the image with the detected circles
def show_image(im_RGB, ballClass):
    # Place the circles in the RGB image
    for val in ballClass:
        ballColor = (255, 0, 0) if val.type == "B" else (255, 255, 255) if val.type == "W" else (0, 0, 255) if val.type == "R" else (10, 116, 245) if val.type == "G" else (10, 30, 55) if val.type == "H" else (0, 0, 0)
        cv.ellipse(im_RGB, val.coordinate, (int(val.minorAxis), int(val.majorAxis)), val.angle, 0, 360, ballColor, 2)
        
    # Show the RGB image with detected circles
    cv.namedWindow('Detected circles', cv.WINDOW_NORMAL)
    cv.imshow('Detected circles', im_RGB)
    cv.waitKey(0)
    cv.destroyAllWindows()

# Debugging
if __name__ == "__main__":
    fileIdx = 0
    InternMat = np.array([[2549.54 / 4, 0, 1639.73 / 4],
                          [0, 2552.31 / 4, 1197.41 / 4],
                          [0, 0, 1]])
    distortion = np.array([0.18, -0.31, 0, 0])
    tilt = 11 * np.pi / 180
    ExternMat = np.dot(np.dot(np.array([[1, 0, 0, 0],
                                        [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
                                        [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
                                        [0, 0, 0, 1]]),
                              np.array([[np.cos(np.pi / 2), 0, np.sin(np.pi / 2), 0],
                                        [0, 1, 0, 0],
                                        [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2), 0],
                                        [0, 0, 0, 1]])),
                              np.array([[1, 0, 0, -0.0025],
                                        [0, np.cos(tilt), -np.sin(tilt), 0],
                                        [0, np.sin(tilt), np.cos(tilt), -0.19],
                                        [0, 0, 0, 1]]))
    
    #imagePath = "C:/Users/shaia/Pictures/COCO images/image analysis/"
    #imageFile = ["image6.jpg", "image7.jpg", "image8.jpg", "image9.jpg", "image12.jpg"]
    #chosenColor = [True, False, False, False, False] if (fileIdx == 0 or fileIdx == 1 or fileIdx == 2) else [False, True, False, True, False] if fileIdx == 3 else [False, False, False, False, True] if fileIdx == 4 else [True, True, True, True, True]
    #ballAmount = [1, 0, 0, 0, 0] if (fileIdx == 0 or fileIdx == 1 or fileIdx == 2) else [0, 1, 0, 2, 0] if fileIdx == 3 else [0, 0, 0, 0, 1] if fileIdx == 4 else [1, 1, 3, 1, 1]
    imagePath = "C:/Users/shaia/Pictures/COCO images/downscale_images/"
    imageFile = ["image01.jpg", "image02.jpg", "image03.jpg", "image04.jpg", "image05.jpg", "image06.jpg", "image07.jpg", "image08.jpg", "image09.jpg"]
    chosenColor = [True, False, False, False, False] if (fileIdx == 0 or fileIdx == 1 or fileIdx == 2 or fileIdx == 3) else [False, False, False, False, True] if (fileIdx == 4 or fileIdx == 5) else [False, True, False, True, False] if (fileIdx == 6 or fileIdx == 7 or fileIdx == 8) else [True, True, True, True, True]
    ballAmount = [1, 0, 0, 0, 0] if (fileIdx == 0 or fileIdx == 1 or fileIdx == 2 or fileIdx == 3) else [0, 0, 0, 0, 1] if (fileIdx == 4 or fileIdx == 5) else [0, 1, 0, 1, 0] if (fileIdx == 6 or fileIdx == 7 or fileIdx == 8) else [1, 1, 3, 1, 1]

    filePath = imagePath + imageFile[fileIdx]
    img = cv.imread(filePath)
    ball = findBall(img, colors = chosenColor)
    if ball is None:
        exit(0)
    foundBall = remove_extra_ball(ball, ballAmount)
    #ballSelect = min(foundBall, key = lambda x: x.coordinate[0])
    #print(get_ball_info(img, ballSelect))
    show_image(img, foundBall)
    ball3D = place3D(InternMat, ExternMat, distortion, foundBall)
    print(ball3D)
    print(next_action(ball3D, 0.25))
