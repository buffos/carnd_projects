import numpy as np
import cv2
from imagetools import ImageChannels, Filters, PipeLine
from camera import Camera
from overlayManager import OverlayManager
from global_variables import DEGREE
from lanesManager import Lanes


# region Class RoadManager
# noinspection PyPep8Naming
class RoadManager:
    def __init__(self):
        self.frame = ImageChannels()  # the Image Channel from the video feed
        self.uframe = ImageChannels()  # the unwrapped Image Channel. Here i will store the unwrapped state of the image
        self.filters = Filters()
        self.pipe = PipeLine(self.frame, self.filters)
        self.cam = Camera()
        self.overlay = OverlayManager()
        self.lanes = Lanes()
        self.X = None
        self.Y = None

    def initData(self, calibrationFilename):
        self.cam.loadCalibration(calibrationFilename)

        self.frame.defineChannel('HSV.V', 'toChannel', 'HSV', 'V')
        self.frame.defineChannel('HSV.H', 'toChannel', 'HSV', 'H')
        self.frame.defineChannel('HSV.S', 'toChannel', 'HSV', 'S')
        self.frame.defineChannel('Side15_RGB.R', 'sideOperator', 15, 'RGB', 'R')
        self.frame.defineChannel('Side15_RGB.G', 'sideOperator', 15, 'RGB', 'G')
        self.frame.defineChannel('Side15_RGB.B', 'sideOperator', 15, 'RGB', 'B')
        self.frame.defineChannel('RGB.R', 'toChannel', 'RGB', 'R')
        self.frame.defineChannel('RGB.G', 'toChannel', 'RGB', 'G')
        self.frame.defineChannel('RGB.B', 'toChannel', 'RGB', 'B')
        self.frame.defineChannel('Preconditioner', 'sideOperator', 31, 'RGB')

        self.uframe.defineChannel('HSV.V', 'toChannel', 'HSV', 'V')
        self.uframe.defineChannel('HSV.H', 'toChannel', 'HSV', 'H')
        self.uframe.defineChannel('HSV.S', 'toChannel', 'HSV', 'S')
        self.uframe.defineChannel('Side15_RGB.R', 'sideOperator', 15, 'RGB', 'R')
        self.uframe.defineChannel('Side15_RGB.G', 'sideOperator', 15, 'RGB', 'G')
        self.uframe.defineChannel('Side15_RGB.B', 'sideOperator', 15, 'RGB', 'B')
        self.uframe.defineChannel('RGB.R', 'toChannel', 'RGB', 'R')
        self.uframe.defineChannel('RGB.G', 'toChannel', 'RGB', 'G')
        self.uframe.defineChannel('RGB.B', 'toChannel', 'RGB', 'B')
        self.uframe.defineChannel('Preconditioner', 'sideOperator', 31, 'RGB')

        self.filters['absoluteFilter'] = self.filters.L1Channel
        self.filters['magnitudeFilter'] = self.filters.L2Channel
        self.filters['directionGradientFilter'] = self.filters.gradientDirection
        self.filters['identity'] = self.filters.identity

        self.pipe.setPipe('lightColorsMask',
                          ('identity', 0, 180, 'HSV.H'),
                          ('identity', 0, 255, 'HSV.S'),
                          ('identity', 220, 255, 'HSV.V'),
                          action='AND'
                          )

        self.pipe.setPipe('lightColorsInShadowsMask',
                          ('identity', 0, 180, 'HSV.H'),
                          ('identity', 0, 30, 'HSV.S'),
                          ('identity', 160, 255, 'HSV.V'),
                          action='AND'
                          )

        self.pipe.setPipe('projectionMask',
                          ('identity', 0, 180, 'HSV.H'),
                          ('identity', 10, 20, 'HSV.S'),
                          ('identity', 100, 255, 'HSV.V'),
                          action='AND'
                          )

        self.pipe.setPipe('ExperimentalMask',
                          ('identity', 0, 10, 'Side15_RGB.R'),
                          ('identity', 0, 30, 'Side15_RGB.G'),
                          ('identity', 5, 50, 'Side15_RGB.B'),
                          action='AND'
                          )

        self.pipe.setPipe('WhiteMask',
                          ('identity', 170, 255, 'RGB.R'),
                          ('identity', 170, 255, 'RGB.G'),
                          ('identity', 170, 255, 'RGB.B'),
                          action='AND'
                          )

        self.pipe.setPipe('YellowMask',
                          ('identity', 10, 40, 'HSV.H'),
                          ('identity', 30, 255, 'HSV.S'),
                          ('identity', 0, 255, 'HSV.V'),
                          action='AND'
                          )

        self.pipe.setPipe('WhiteMask2',
                          ('identity', 220, 255, 'RGB.R'),
                          ('identity', 220, 255, 'RGB.G'),
                          ('identity', 220, 255, 'RGB.B'),
                          action='AND'
                          )

        self.pipe.setPipe('YellowMask2',
                          ('identity', 10, 40, 'HSV.H'),
                          ('identity', 30, 255, 'HSV.S'),
                          ('identity', 0, 255, 'HSV.V'),
                          ('identity', 200, 255, 'RGB.R'),
                          ('identity', 200, 255, 'RGB.G'),
                          action='AND'
                          )

    def setFrame(self, image, unwrapped=False):
        if unwrapped is True:
            self.uframe.setImage(image)
        else:
            self.frame.setImage(image)
        self.X = self.frame.image.shape[1]
        self.Y = self.frame.image.shape[0]

    def loadFrame(self, filename):
        self.frame.loadImage(filename)
        self.X = self.frame.image.shape[1]
        self.Y = self.frame.image.shape[0]

    def getFrame(self):
        return self.frame.image

    def identifyLanes(self, mask):
        histogram = self.uframe.histogram(mask)
        newHistogram, lanePeaks = self.uframe.removeNoise(histogram)
        return newHistogram, lanePeaks

    def processFrame(self, image=None, imageFile=None):
        if imageFile is None:
            self.setFrame(image)
        else:
            self.loadFrame(imageFile)

        # step 1: undistort image
        undistortedImage = self.cam.undistortImage(self.getFrame())
        self.setFrame(undistortedImage)
        # # step 2: the orthogonal projection of the image
        unwrappedImage = self.cam.toOrthogonal(undistortedImage)
        self.setFrame(unwrappedImage, unwrapped=True)  # setting the unwrapped bird's eye vies
        # step 3: apply filters
        self.pipe.changeImageChannel(self.uframe)  # set the filter pipe to run on the unwrapped frame
        # mask = self.pipe.run('lightColorsMask')
        # mask = self.pipe.run('ExperimentalMask')
        # self.uframe.image = self.uframe.getChannel('Preconditioner')
        mask = self.pipe.run('WhiteMask') | self.pipe.run('YellowMask')

        lightColorsDirGradientMask = \
            self.filters['directionGradientFilter'](70 * DEGREE,
                                                    120 * DEGREE,
                                                    cv2.Sobel(mask, cv2.CV_64F, 1, 1, ksize=3),
                                                    cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3))

        lightColorsMagGradientMask = \
            self.filters['magnitudeFilter'](120, 255,
                                            cv2.Sobel(mask, cv2.CV_64F, 1, 0, ksize=3),
                                            cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3))
        return mask

    @staticmethod
    def houghLines(img, rho, theta, threshold, min_line_len, max_line_gap):
        """
        `img` should be the output of a Canny transform.

        Returns an image with hough lines drawn.
        """
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                                maxLineGap=max_line_gap)
        return lines

        # endregion
