import csv
import numpy as np
import cv2
import sys
from matplotlib import pyplot as plt

def readData(filename):
    """
        Reads the data shaped like 600 x 10 (N x F)
        600 X or Y points in 10 frames
    """

    tmpList = []

    with open(filename, newline='') as csvfile:
        csvReader = csv.reader(csvfile, delimiter = ",")
        tmpList = list(csvReader)

    return np.array(tmpList, dtype = np.float)

def centerPoints(framesList):
    """
        To apply SFM, find the mass of the points in each frame.
        Substract the mass from each point to get centered the points.
    """

    frameNum, pointNum = framesList.shape

    centeredPoints = np.zeros((frameNum, pointNum), dtype = np.float)
    translationElementOfFrame = np.zeros((frameNum), dtype = np.float)
    for i in range(frameNum):
        pointsInFrame = framesList[i]
        massOfPoints = np.mean(pointsInFrame, dtype = np.float)
        translationElementOfFrame[i] = massOfPoints

        for j, point in enumerate(pointsInFrame):
            centeredPoint = point - massOfPoints
            centeredPoints[i][j] = centeredPoint

    return centeredPoints, translationElementOfFrame

def createMeasurementMatrix(xList, yList):
    """
        Create the measurement matrix in shape of 2F x N (20 x 600)
    """

    frameNum, pointNum = xList.shape
    measurementMatrix = np.zeros((2 * frameNum, pointNum), dtype = np.float)

    # put x points into first half and y points into second half
    measurementMatrix[: frameNum] = xList
    measurementMatrix[frameNum: ] = yList

    return measurementMatrix

def getCameraMatrix(motionMatrix, whichFrame):
    """
        Get the required camera matrix from the motion matrix after SVD decompostion
    """
    firstRow  = motionMatrix[whichFrame]
    secondRow = motionMatrix[whichFrame + 10]

    m = np.concatenate((firstRow, secondRow), axis = 0)
    return np.reshape(m, (2, 3))

def getTanslationVector(xTranslationInFrames, yTranslationInFrames, whichFrame):
    """
        Get the translation vector of the required camera i.e frame
    """

    xTi = xTranslationInFrames[whichFrame]
    yTi = yTranslationInFrames[whichFrame]

    ti = np.array([[xTi], [yTi]])
    return ti

def createProjectionMatrix(cameraMatrix, translationVector):
    """
        Create the affine projection matrix 3 x 4.
        P = ((Mi, ti),
             (0,  1))
    """

    upperSideP = np.concatenate((cameraMatrix, translationVector), axis = 1)
    bottomP = np.array([[0, 0, 0, 1]])

    P = np.concatenate((upperSideP, bottomP), axis = 0)
    return P

def applyProjectPoints(cameraMatrix, worldPoints):
    """
        Applies the project points function to projec the
        3D world points into an image plane
    """
    row, col = worldPoints.shape
    m = np.concatenate((cameraMatrix, np.array([[0, 0, 1]])))

    rvec = np.array([0, 0, 0], dtype = np.float)
    tvec = np.array([0, 0, 1], dtype = np.float)

    points, _ = cv2.projectPoints(worldPoints, rvec, tvec, m, ())
    points = np.reshape(points, (row, 2))

    return points

def getWorldAndImageCoordinates(whichFrame):
    """
        Read the X and Y points.
        Create measurement matrix.
        Apply SVD decompositon to get the required matrices.
        Calculate the Motion and Structure matrices.
        Calculate the projection matrix.
        Project the 3d world points to an image plane.

        Return both 3d world points and projected points.
    """

    xList = readData("xPoints.csv")
    yList = readData("yPoints.csv")

    xList = np.transpose(xList)
    yList = np.transpose(yList)

    xList, xTranslationInFrames = centerPoints(xList)
    yList, yTranslationInFrames = centerPoints(yList)

    measurementMatrix = createMeasurementMatrix(xList, yList)

    # apply svd decomposition
    w, u, vt = cv2.SVDecomp(measurementMatrix)

    # calculate both motion and structure matrices
    singularValues = np.array([[w[0][0], 0, 0], [0, w[1][0], 0], [0, 0, w[2][0]]])

    # get the world points, i.e. structure, as the first the column of vt matrix
    v = np.transpose(vt)
    worldPoints = v[:, :3]

    # get the camera matrices, i.e. motion matrices
    motionMatrix = np.matmul(u[:, :3], singularValues)

    # get projection matrix of the required frame
    frameIndex = whichFrame
    cameraMatrix = getCameraMatrix(motionMatrix, frameIndex)
    translationVector = getTanslationVector(xTranslationInFrames, yTranslationInFrames, frameIndex)
    P = createProjectionMatrix(cameraMatrix, translationVector)

    # apply projectPoints function to project the 3D points into 2D image plane
    pointsFromProjectionMatrix = applyProjectPoints(cameraMatrix, worldPoints)

    # project world coordinates to image plane using the projection matrix
    imageCoordinates = []
    for point in worldPoints:
        homogenousPoint = np.concatenate((point, [1]), axis = 0)
        coordinate = np.matmul(P, homogenousPoint)

        x = coordinate[0] / coordinate[2]
        y = coordinate[1] / coordinate[2]

        coordinate = np.array([x, y])
        imageCoordinates.append(coordinate)

    return worldPoints, imageCoordinates, pointsFromProjectionMatrix

def showWithScatter(points):
    """
        Plot the points retrieved from projectPoints function
    """
    x = points[:, 0]
    y = points[:, 1]

    plt.plot(x, y,  color = "black")
    plt.show()

def show2D(imageCoordinates):
    """
        Plot the projected world points to image plane.
    """
    img = np.zeros((1000, 1000))

    for coordinate in imageCoordinates:
        x, y = 400 * coordinate + 500
        x = int(x)
        y = int(y)
        cv2.circle(img, (y, x), 5, (255, 255, 255))

    plt.imshow(img)
    plt.show()

def show3D(worldPoints):
    """
        Plot the reconstructred figure
    """
    fig = plt.figure()
    ax = plt.axes(projection = "3d")

    xPts = worldPoints[:,0]
    yPts = worldPoints[:,1]
    zPts = worldPoints[:,2]

    ax.scatter3D(xPts, yPts, zPts, c = zPts, cmap = "ocean")
    plt.show()

frameNum = 4
worldPoints, imageCoordinates, pointsFromProjectionMatrix = getWorldAndImageCoordinates(frameNum)

showWithScatter(pointsFromProjectionMatrix)
show2D(imageCoordinates)
show3D(worldPoints)
