# Libraries
import cv2 as cv
import numpy as np

# Modules
#from Robot import robot

# Detection of ArUco codes
def detect_aruco(im_RGB):
    # Getting the image as RGB and gray-scaled image
    im_gray = cv.cvtColor(im_RGB, cv.COLOR_BGR2GRAY)

    # Getting the predefined dictionary and parameters of ArUco codes
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters_create()

    # Detection of corner, id and rejection of ArUco codes
    corners, ids, _ = cv.aruco.detectMarkers(im_gray, aruco_dict, parameters=parameters)
    
    # Return the id and corners
    return ids, corners

# Finding the rotation and translation matrix of the detected ArUco codes
def orientation(corners, distortion, cameraCalMatrix, markerLength = 0.1):
    # The object points of the real-world ArUco codes
    objectPoints = np.array([[-markerLength / 2,  markerLength / 2, 0],
                             [ markerLength / 2,  markerLength / 2, 0],
                             [ markerLength / 2, -markerLength / 2, 0],
                             [-markerLength / 2, -markerLength / 2, 0]])
    # No distortion in camera
    distCoeffs = distortion
    
    # Finding the rotation and translation matrix
    imagePoints = corners.reshape(-1, 2)
    success, rot, trans = cv.solvePnP(objectPoints, imagePoints, cameraCalMatrix, distCoeffs)
    if success:
        return rot, trans
    else:
        return None, None    

# Get the camera position in the world coordinate system
def get_camera_position(rot, trans):
    # Go through the detected ArUco codes
    Rot, _ = cv.Rodrigues(rot)
    marker_pos = -np.dot(np.transpose(Rot), trans)

    # Find the angle and distance to travel
    angle = np.arctan2(marker_pos[0], marker_pos[2]) * 180 / np.pi
    distance = np.linalg.norm(marker_pos[[0, 2]])
    
    # Return the position angle and distance
    return marker_pos, angle, distance

# Get the position to move towards the ArUco marker
def get_position(pos, turnRadius = 0.4):
    # Find the angle to turn
    angle = np.arctan2(pos[0][0], pos[2][0])

    # The robot will first turn, so new position has to be re-calculated
    R = np.array([[np.cos(angle), 0, np.sin(angle)],
                   [0, 1, 0],
                   [-np.sin(angle), 0, np.cos(angle)]])
    T = np.array([turnRadius * np.sin(angle), 0, turnRadius * (1 - np.cos(angle))]).reshape(3, 1)

    marker_newpos = np.dot(np.transpose(R), pos - T)

    # Find the distance to travel
    distance = np.linalg.norm(marker_newpos[[0, 2]])
    
    # Return the angle and distance
    return angle * 180 / np.pi, distance

# Get information for the code in robocupmain.py
def get_aruco_info(img, corner):
    # Getting the frame width
    frame_width = img.shape[1]

    # Getting the ball x center
    upperX = abs(corner[0][0] - corner[1][0]) / 2
    lowerX = abs(corner[2][0] - corner[3][0]) / 2
    betweenX = abs(upperX - lowerX) / 2
    x_center = int(corner[0][0] + upperX - betweenX)

    # Getting the area of the marker
    size = int(abs(corner[0][0] - corner[1][0]) * abs(corner[2][0] - corner[3][0]))

    # Return the frame_width, x center and size of the ball
    return frame_width, x_center, size

# Show the images  with the detected ArUco codes
def show_image(im_RGB, ids, corners, rot, trans, cameraCalMatrix):
    # All the detected ID
    if ids is not None:
        cv.aruco.drawDetectedMarkers(im_RGB, corners, ids)
        distCoeffs = np.zeros((4, 1))
        # Draw the axes of the detected ArUco codes
        for idx, idElem in enumerate(ids):
            cv.drawFrameAxes(im_RGB, cameraCalMatrix, distCoeffs, rot[idx], trans[idx], 0.05, 5)
            corner_point = tuple(map(int, corners[idx][0][0]))
            cv.putText(im_RGB, f'ID: {idElem[0]}', corner_point, cv.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3, cv.LINE_AA)
        # Show the image
        cv.namedWindow('Detected Markers', cv.WINDOW_NORMAL)
        cv.imshow('Detected Markers', im_RGB)
        cv.waitKey(0)
        cv.destroyAllWindows()

# Debugging
if __name__ == "__main__":
    fileIdx = 4
    InternMat = np.array([[2549.54 / 4, 0, 1639.73 / 4],
                          [0, 2552.31 / 4, 1197.41 / 4],
                          [0, 0, 1]])
    distortion = np.array([0.18, -0.31, 0, 0])
    #path = "C:/Users/shaia/Pictures/COCO images/image analysis/"
    #imageFile = ["image1.jpg", "image2.jpg", "image3.jpg", "image4.jpg", "image5.jpg", "image9.jpg", "image10.jpg", "image11.jpg"]
    path = "C:/Users/shaia/Pictures/COCO images/downscale_images/"
    imageFile = ["image10.jpg", "image11.jpg", "image12.jpg", "image13.jpg", "image14.jpg"]
    filePath = path + imageFile[fileIdx]
    img = cv.imread(filePath)
    arucoID, corner = detect_aruco(img)
    #arucoSelect = [point[0] for point in corner for point in point for point in point]
    #cornerSelect = corner[np.argmin(arucoSelect)][0]
    #print(get_aruco_info(img, cornerSelect))
    rotation, translation = orientation(corner, distortion, InternMat)
    show_image(img, arucoID, corner, rotation, translation, InternMat)
    print(get_camera_position(rotation, translation))
