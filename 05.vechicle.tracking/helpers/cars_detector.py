# from sklearn.preprocessing import StandardScaler
# from sklearn.model_selection import train_test_split
# from sklearn.svm import LinearSVC
# from sklearn.calibration import CalibratedClassifierCV
from sklearn.externals import joblib  # it claims to be faster the pickle

from helpers.imagetools import ImageChannels
from helpers.global_variables import *
from helpers.camera import Camera

import numpy as np
import cv2

HOG_VECTORS = 3
CONFIDENCE_THRESHOLD = 0.8
VOTE_THRESHOLD = 2
FONT_SCALE = 0.6
FONT_THICKNESS = 1
FONT_FAMILY = cv2.FONT_HERSHEY_DUPLEX


# noinspection PyPep8Naming
class CarDetector:
    def __init__(self):
        # Load the classifier model
        self.svc = joblib.load('./models/svc.p')
        self.svc_calibrated = joblib.load('./models/svc_calibrated.p')

        # Load the standard scalar model
        self.X_scaler = joblib.load('./models/scv_data_scaler.p')

        # scanning regions
        self.regions = {'large': {'ys': (400, 600), 'xs': (400, 1280), 'size': 96, 'overlap': 0.7},  # 99 windows
                        'medium': {'ys': (400, 600), 'xs': (0, 1280), 'size': 128, 'overlap': 0.9},  # 584 windows
                        'small': {'ys': (300, 500), 'xs': (400, 1280), 'size': 64, 'overlap': 0.6},  # 1683 windows
                        }
        self.confidenceThreshold = CONFIDENCE_THRESHOLD

    # region classification
    def classifyBoundingBox(self, resizedImage, hogVector, boundingBox):
        # boundingBox[y][x]
        # boxes are supposed to be from 64x64 images. So no resizing
        # the box coordinates are CELL coordinates
        startX, startY, endX, endY = boundingBox
        # i have 3 hogVectors
        hog_length = np.int(hogVector.shape[0] / HOG_VECTORS)
        subHog = np.array([])
        for i in range(HOG_VECTORS):
            subHog = np.concatenate([subHog,
                                     hogVector[startY + i * hog_length:endY + i * hog_length, startX:endX].ravel()]
                                    )

        # calculate actual coordinate for the resized image
        # this is why scale is 1
        startX, startY, endX, endY = self.cellToImageCoordinates(boundingBox, (1, 1))

        subImage = resizedImage[startY:endY, startX:endX]

        tempFrame = ImageChannels()
        tempFrame.setImage(subImage)

        fv_histogram = tempFrame.generateFeaturesVector(histogramsChannels=['HLS.S'])
        fv = np.concatenate([fv_histogram, subHog]).reshape(1, -1)

        fv_scaled = self.X_scaler.transform(fv)  # normalize feature vector

        prediction = self.svc_calibrated.predict(fv_scaled)
        confidence_level = self.svc_calibrated.predict_proba(fv_scaled)

        isCar = True if prediction == 1 else False

        return isCar, confidence_level

    def classifyBoundingBoxList(self, resizedImage, hogVector, boundingBoxList, threshold=0.8):
        boxesWithCars = []

        for box in boundingBoxList:
            isCar, probabilities = self.classifyBoundingBox(resizedImage, hogVector, box)
            if isCar is True and probabilities[0][1] >= threshold:
                boxesWithCars.append([box, probabilities[0][1]])

        return boxesWithCars

    # endregion

    # region coordinate transformation

    @staticmethod
    def cellToImageCoordinates(box, scale, offset=(0, 0)):
        # the correct equation between cells an image pixel is
        # X/PPC - CPB + 1 = CELL
        # and solving for X
        # X = (CELL-1 + CPB) * PPC
        # using offset ONLY when i want to restore the global coordinates
        # if i used an imageStripe
        # offset = (xs,ys)
        startX, startY, endX, endY = box

        startX = np.int((startX - 2 + CPB) * PPC * scale[1])
        endX = np.int((endX - 1 + CPB) * PPC * scale[1])
        startY = np.int((startY - 2 + CPB) * PPC * scale[0])
        endY = np.int((endY - 1 + CPB) * PPC * scale[0])
        return startX + offset[0], startY + offset[1], endX + offset[0], endY + offset[1]

    @staticmethod
    def imageToCellCoordinate(box, scale):
        startX, startY, endX, endY = box

        startX = np.int(startX / (PPC * scale[1]) - CPB + 1)
        endX = np.int(endX / (PPC * scale[1]) - CPB + 1)
        startY = np.int(startY / (PPC * scale[0]) - CPB + 1)
        endY = np.int(endY / (PPC * scale[0]) - CPB + 1)
        return startX, startY, endX, endY

    @staticmethod
    def boundingBoxesToImageCoordinates(results, scaleFactor, offset):
        """
        Results are pairs of bounding boxes and confidence score.
        Bounding boxes are in cell coordinates, so they have to be transferred back to image ones
        To do that we want the scale factor that was used to resized the initial image since
        we target (64,64) cells, if originally we had (256,256) sells we downsized the image by 4 so it has
        to be up sized. Also offset uses the fact the the image is a stripe of the original one, so if for
        example we start from y=250px of the original image, that is the y-offset
        :param results:
        :param scaleFactor:
        :param offset:
        :return:
        """
        for result in results:
            result[0] = CarDetector.cellToImageCoordinates(result[0], scaleFactor, offset)
        return results

    # endregion

    # region drawing
    @staticmethod
    def draw_boxes(image, box, message):
        cv2.rectangle(image, pt1=box[0:2], pt2=box[2:], color=(0, 0, 255), thickness=2)
        for i, line in enumerate(str(message).split('\n')[::-1]):
            anchorText = (box[0], box[1] - i * 20)  # print text..down up 5 pixels at at time
            cv2.putText(image, line, anchorText, fontFace=FONT_FAMILY, fontScale=FONT_SCALE, color=(0, 255, 255),
                        thickness=FONT_THICKNESS)

    @staticmethod
    def draw_all_boxes(image, results, scaleFactor=None, offset=None):
        for result in results:
            if (scaleFactor is not None) and (offset is not None):
                box = CarDetector.cellToImageCoordinates(result[0], scaleFactor, offset)
            else:
                box = result[0]

            box = tuple(box)
            confidence = result[1]
            CarDetector.draw_boxes(image, box, confidence)

    # endregion

    # region slidingWindow
    @staticmethod
    def slidingWindowInStripe(image,
                              ys=(None, None),
                              xs=(None, None),
                              slidingWindowSize=(256, 256),
                              overlap=(0.5, 0.5)):
        # data initialisation
        if xs[0] is None:
            xs = (0, xs[1])
        if xs[1] is None:
            xs = (xs[0], image.shape[1])
        if xs[1] - xs[0] < slidingWindowSize[1]:
            xs = (xs[0], xs[0] + slidingWindowSize[1])
        if ys[0] is None:
            ys = (0, ys[1])
        if ys[1] is None:
            ys = (ys[0], image.shape[0])
        if ys[1] - ys[0] < slidingWindowSize[0]:
            ys = (ys[0], ys[0] + slidingWindowSize[0])

        scaleFactor = (slidingWindowSize[0] / SAMPLE_SHAPE[0], slidingWindowSize[1] / SAMPLE_SHAPE[1])

        # Resizing image so for example (256,256) will become (64,64) images in the new resizes imaged
        # So when i compute the features hog vector for the whole resized image and then take
        # (7,7,2,2,9) slices from that vector, those will correspond to (256,256) sliding windows
        # the benefit is I do not have to calculate the hog feature vector again and again
        stripeWidth = xs[1] - xs[0]
        stripeHeight = ys[1] - ys[0]

        newWidth = int(stripeWidth / scaleFactor[0])
        newHeight = int(stripeHeight / scaleFactor[1])

        imageStripe = image[ys[0]:ys[1], xs[0]:xs[1]]
        resizedStripe = cv2.resize(imageStripe, (newWidth, newHeight))
        tempFrame = ImageChannels()
        tempFrame.setImage(resizedStripe)

        # fv_histogram = tempFrame.generateFeaturesVector(histogramsChannels=['HLS.S', 'HLS.S'])
        fv_hog = tempFrame.generateFeaturesVector(hogChannels=['YCrCb.Y', 'YCrCb.Cr', 'YCrCb.Cb'],
                                                  featureVector=False)

        n_yBlocks = fv_hog.shape[0] / HOG_VECTORS  # hogs are concatenated on axis=0
        n_xBlocks = fv_hog.shape[1]

        cellsPerImageX = PPC - 1  # 7
        cellsPerImageY = PPC - 1  # 7

        yBlockStep = max(1, np.int(cellsPerImageY * (1. - overlap[0])))
        xBlockStep = max(1, np.int(cellsPerImageX * (1. - overlap[1])))

        # number of sliding windows in the X and Y direction
        # calculate the space left and divide by the step
        # add one because we always have the window we subtracted
        ny_windows = np.int((n_yBlocks - cellsPerImageY) / yBlockStep) + 1
        nx_windows = np.int((n_xBlocks - cellsPerImageX) / xBlockStep) + 1

        slidingWindowsList = []

        # these are coordinate for the fv_hog vector
        for xs in range(nx_windows):
            for ys in range(ny_windows):
                startX = xs * xBlockStep
                endX = startX + cellsPerImageX
                startY = ys * yBlockStep
                endY = startY + cellsPerImageY
                slidingWindowsList.append((startX, startY, endX, endY))

        return np.array(slidingWindowsList), fv_hog, resizedStripe, scaleFactor

    # endregion

    def detectCars(self, image):
        detectedCars = []
        for key, region in self.regions.items():
            results = self.slidingWindowInStripe(image,
                                                 ys=region['ys'],
                                                 xs=region['xs'],
                                                 slidingWindowSize=(region['size'], region['size']),
                                                 overlap=(region['overlap'], region['overlap']))

            slidingWindowsList, vector_hog, resizedImg, scaleFactor = results

            possibleCars = self.classifyBoundingBoxList(resizedImg,
                                                        hogVector=vector_hog,
                                                        boundingBoxList=slidingWindowsList,
                                                        threshold=self.confidenceThreshold)

            possibleCars = self.boundingBoxesToImageCoordinates(possibleCars,
                                                                scaleFactor=scaleFactor,
                                                                offset=(region['xs'][0], region['ys'][0]))

            detectedCars += possibleCars

        return detectedCars

    # Malisiewicz et al.
    # https://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/
    @staticmethod
    def non_max_suppression_fast(results, overlapThresh):
        # get the bounding boxes and transform them to a 4 column array
        if results is None or len(results) == 0:
            return []

        boxes = np.concatenate(np.array(results)[:, 0]).reshape(-1, 4)
        confidences = np.array(results)[:, 1]

        # if there are no boxes, return an empty list
        if len(boxes) == 0:
            return []

        # if the bounding boxes integers, convert them to floats --
        # this is important since we'll be doing a bunch of divisions
        if boxes.dtype.kind == "i":
            boxes = boxes.astype("float")

        # initialize the list of picked indexes
        pick = []

        # grab the coordinates of the bounding boxes
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        # compute the area of the bounding boxes and sort the bounding
        # boxes by the bottom-right y-coordinate of the bounding box
        area: np.ndarray = (x2 - x1 + 1) * (y2 - y1 + 1)
        indexes = np.argsort(y2)

        # keep looping while some indexes still remain in the indexes
        # list
        while len(indexes) > 0:
            # grab the last index in the indexes list and add the
            # index value to the list of picked indexes
            last = len(indexes) - 1
            i = indexes[last]
            pick.append(i)

            # find the largest (x, y) coordinates for the start of
            # the bounding box and the smallest (x, y) coordinates
            # for the end of the bounding box
            xx1 = np.maximum(x1[i], x1[indexes[:last]])
            yy1 = np.maximum(y1[i], y1[indexes[:last]])
            xx2 = np.minimum(x2[i], x2[indexes[:last]])
            yy2 = np.minimum(y2[i], y2[indexes[:last]])

            # compute the width and height of the bounding box
            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)

            # compute the ratio of overlap
            overlap = (w * h) / area[indexes[:last]]

            # delete all indexes from the index list that have
            indexesToMerge = np.concatenate(([last], np.where(overlap > overlapThresh)[0]))
            overlappingBoxes = len(indexesToMerge)
            if overlappingBoxes < VOTE_THRESHOLD:
                pick.pop()
            else:
                xb1 = np.amin(x1[indexes[indexesToMerge]])
                yb1 = np.amin(y1[indexes[indexesToMerge]])
                xb2 = np.amax(x2[indexes[indexesToMerge]])
                yb2 = np.amax(y2[indexes[indexesToMerge]])

                boxes[pick[-1]] = xb1, yb1, xb2, yb2

            indexes = np.delete(indexes, indexesToMerge)

        # return only the bounding boxes that were picked using the
        # integer data type
        return list(zip(boxes[pick].astype(np.int), confidences[pick]))

    def __del__(self):
        pass


if __name__ == '__main__':
    import time

    cd = CarDetector()
    frame = ImageChannels()
    cam = Camera()

    frame.loadImage('../test_images/test5.jpg')
    someImage = frame.image.copy()

    t_start = time.time()
    detected = cd.detectCars(someImage)
    t_end = time.time()
    print("Time per frame {0:.2f} sec".format(t_end - t_start))
    picks = cd.non_max_suppression_fast(detected, 0.1)
    print(detected)
    # cd.draw_all_boxes(someImage,detected)

    cd.draw_all_boxes(someImage, picks)

    cv2.imshow('', someImage)
    cv2.waitKey(0)
