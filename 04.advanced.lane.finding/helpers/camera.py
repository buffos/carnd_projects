import numpy as np
import cv2
import pickle
import glob


# region Camera Class
# noinspection PyPep8Naming
class Camera:
    def __init__(self):
        self.object_points = []
        self.image_points = []
        self.cameraMatrix = {}
        self.distortionCoefficient = {}
        self.perspectiveMatrix = None
        self.srcPoints = None  # last src perspective  used to create the Matrix
        self.dstPoints = None
        self.lanePerImage = 4  # how many lanes to squeeze in when performing perspective transformation

    # region Load Save Reset Calibration data
    def loadCalibration(self, filename):
        try:
            stored = pickle.load(open(filename, 'rb'))
            self.object_points = stored['object_points']
            self.image_points = stored['image_points']
            self.cameraMatrix = stored['cameraMatrix']
            self.distortionCoefficient = stored['distortionCoefficient']
            # self.perspectiveMatrix = stored['perspectiveMatrix']
        except Exception as e:
            print(e)

    def saveCalibration(self, filename):
        stored = {'object_points': self.object_points,
                  'image_points': self.image_points,
                  'cameraMatrix': self.cameraMatrix,
                  'distortionCoefficient': self.distortionCoefficient,
                  # 'perspectiveMatrix': self.perspectiveMatrix
                  }
        pickle.dump(stored, open(filename, 'wb'))

    def resetCalibration(self):
        self.object_points = []
        self.image_points = []
        self.cameraMatrix = {}
        self.distortionCoefficient = {}
        self.perspectiveMatrix = None

    # endregion

    # region  Calibration Functions
    def calibrateFromImages(self, globPattern, boardCornersShape, showResults=True):
        self.resetCalibration()

        objectGrid = np.zeros((boardCornersShape[0] * boardCornersShape[1], 3), np.float32)
        objectGrid[:, :2] = np.mgrid[0: boardCornersShape[0], 0: boardCornersShape[1]].T.reshape(-1, 2)
        images = glob.glob(globPattern)
        noOfImage = len(images)

        for index, filename in enumerate(images):
            print('Processing Image {0} of {1}'.format(index + 1, noOfImage))
            image = cv2.imread(filename)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            found, corners = cv2.findChessboardCorners(gray, (boardCornersShape[0], boardCornersShape[1]), None)

            # If found, add object points, image points
            if found is True:
                self.object_points.append(objectGrid)
                self.image_points.append(corners)

                # Draw and display the corners
                if showResults is True:
                    cv2.drawChessboardCorners(image,
                                              patternSize=boardCornersShape,
                                              corners=corners,
                                              patternWasFound=found)
                    cv2.imshow('img', image)
                    cv2.waitKey(500)

        if showResults is True:
            cv2.destroyAllWindows()

    def calibrate(self, imageSize):
        if len(self.object_points) < 5:  # few image calibration data
            print('not enough calibration data')
            return

        ret, cameraMatrix, distortionCoefficient, rotationVectors, translationVectors = cv2.calibrateCamera(
            self.object_points, self.image_points, imageSize, None, None)

        self.cameraMatrix[imageSize] = cameraMatrix
        self.distortionCoefficient[imageSize] = distortionCoefficient

    def calculatePerspectiveMatrix(self, sourcePoints, destinationPoints):
        self.perspectiveMatrix = cv2.getPerspectiveTransform(sourcePoints, destinationPoints)

        return self.perspectiveMatrix

    # endregion

    def undistortImageFromFile(self, filename, save=False):
        image = cv2.imread(filename)
        imageSize = (image.shape[1], image.shape[0])

        if self.cameraMatrix.get(imageSize) is None:  # no cameraMatrix for the imageSize
            self.calibrate(imageSize)

        undistortedImage = cv2.undistort(image,
                                         self.cameraMatrix[imageSize],
                                         self.distortionCoefficient[imageSize],
                                         None,
                                         self.cameraMatrix[imageSize]
                                         )
        if save is True:
            saveName = filename[:filename.rfind('.')] + '_undistorted.' + filename[filename.rfind('.') + 1:]
            cv2.imwrite(saveName, undistortedImage)
        else:
            cv2.imshow('img', undistortedImage)
            cv2.waitKey(0)
            return undistortedImage, image
        return None, image

    def undistortImage(self, image):
        imageSize = (image.shape[1], image.shape[0])
        if self.cameraMatrix.get(imageSize) is None:  # no cameraMatrix for the imageSize
            self.calibrate(imageSize)

        undistortedImage = cv2.undistort(image,
                                         self.cameraMatrix[imageSize],
                                         self.distortionCoefficient[imageSize],
                                         None,
                                         self.cameraMatrix[imageSize]
                                         )
        return undistortedImage

    def toOrthogonal(self, image):
        image_size = (image.shape[1], image.shape[0])  # imageY, imageX
        if self.perspectiveMatrix is None:
            self.initializeProjectionPoints(image)
        return cv2.warpPerspective(image, self.perspectiveMatrix, image_size, flags=cv2.INTER_NEAREST)

    def toProjected(self, image):
        image_size = (image.shape[1], image.shape[0])  # imageY, imageX
        if self.perspectiveMatrix is None:
            self.initializeProjectionPoints(image)
        return cv2.warpPerspective(image, self.perspectiveMatrix, image_size,
                                   flags=cv2.WARP_INVERSE_MAP | cv2.INTER_NEAREST)

    def initializeProjectionPoints(self, image):
        self.srcPoints, self.dstPoints = self.getDefaultPerspectiveCorrectionPoints(image)
        # hardcoded src and destination points
        self.calculatePerspectiveMatrix(self.srcPoints, self.dstPoints)
        # calculate the Perspective Matrix which is now stored inside the cam

    @staticmethod
    def createSourceDestinationPointsFromChessBoard(filename, boardCornersShape):
        objectGrid = np.zeros((boardCornersShape[0] * boardCornersShape[1], 3), np.float32)
        objectGrid[:, :2] = np.mgrid[0: boardCornersShape[0], 0: boardCornersShape[1]].T.reshape(-1, 2)
        image = cv2.imread(filename)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, (boardCornersShape[0], boardCornersShape[1]), None)

        if found is True:
            dest = objectGrid[:, :2].tolist()  # throw away the z coordinate
            source = corners.tolist()
            return np.array([source[0][0], source[1][0], source[9][0], source[10][0]], np.float32), np.array(
                [dest[0], dest[1], dest[9], dest[10]], np.float32)

    def getDefaultPerspectiveCorrectionPoints(self, forImage):
        """
        these are perspective points from image supplied.
        it will be used as a fall back if no perspective calibration points are found
        :param forImage:
        :return:
        """
        imageWidth = forImage.shape[1]  # X coord
        imageHeight = forImage.shape[0]  # Y coord
        x_offset = 50
        lanes = self.lanePerImage

        src = np.array([[220. / 1280. * imageWidth, 719. / 720. * imageHeight],
                        [1120. / 1280. * imageWidth, 719. / 720. * imageHeight],
                        [740. / 1280. * imageWidth, 480. / 720. * imageHeight],
                        [550. / 1280. * imageWidth, 480. / 720. * imageHeight]], np.float32)

        # src = np.float32([[220, 719], [1220, 719], [750, 480], [550, 480]])

        dst = np.array([[x_offset + 0. * imageWidth, 1 * imageHeight],
                        [x_offset + 1 / lanes * imageWidth, 1 * imageHeight],
                        [x_offset + 1 / lanes * imageWidth, 0. * imageHeight],
                        [x_offset + 0. * imageWidth, 0. * imageHeight]], np.float32)
        return src, dst

# endregion
