from .global_variables import *
from .experimentalSideOperator import sideImageFilter, sideChannelFilter

import numpy as np
import cv2
from skimage.feature import hog


# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg


# noinspection PyPep8Naming
class ImageChannels:
    """
    A class to facilitate extracting channel information from an image
    """

    def __init__(self):
        self.image = None  # image always stored in RGB
        self.channels = {}  # in this dictionary we stored partial functions that just require calling with zero args

    # region getters and setters
    def __getitem__(self, name):
        """
        Overloading the [] to get the defined channel name from the corresponding dictionary
        :param name:
        :return: returns a function that is called with zero arguments and which return a channel
        """
        return self.channels[name]

    def __setitem__(self, key, value):
        self.defineChannel(key, value[0], *value[1:])

    def setImage(self, image):
        self.image = image

    def loadImage(self, filename):
        import os
        if not os.path.isfile(filename):
            raise FileExistsError("ImageChannel.loadImage: filename not found")
        else:
            self.image = cv2.imread(filename)
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            # self.image = mpimg.imread(filename)

    def getChannel(self, name):
        return self[name][0](*self[name][1:])

    def defineChannel(self, name, fromFunction, *args):
        """
        Defines a predefined function , aimed to be called with no arguments. Its based on the contained class
        functions, and just has pre configured arguments passed to it, to make it more compact.
        We then put it in a dictionary with a custom meaningful name
        :param name: the key to use in the dictionary
        :param fromFunction: what function to use
        :param args: the arguments that should be passed to the function
        :return: Nothing
        """

        def newChannel(*params):
            return getattr(self, fromFunction)(*params)  # (*args)

        self.channels[name] = (newChannel, *args)

    # endregion

    # region change channels

    def toChannel(self, colorSpace='HLS', channel='L', image=None):
        """
        transforms the image to a new colorSpace and extract the wanted channel information
        :param colorSpace:
        :param channel:
        :param image: you can use an image not stored in the class to do the transformation
        :return: an np.array with the wanted channel
        """
        if image is None:
            image = self.image

        if colorSpace == 'GRAY':
            return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        if colorSpace == 'YCrCb':
            if channel == 'Y':
                channelIndex = 0
            elif channel == 'Cr':
                channelIndex = 1
            else:
                channelIndex = 2
            return cv2.cvtColor(image, cv2.COLOR_RGB2YCrCb)[:, :, channelIndex]

        # find which channel to select
        channelIndex = colorSpace.rfind(channel)
        if channelIndex < 0 or channelIndex > 3:
            return None

        # check if we are in RGB so no conversion needed
        if colorSpace == 'RGB':  # no need for conversion
            return image[:, :, channelIndex]

        newColorSpace = 'cv2.COLOR_RGB2' + colorSpace
        return cv2.cvtColor(image, eval(newColorSpace))[:, :, channelIndex]

    def toSpace(self, colorSpace='HLS', image=None):
        if image is None:
            image = self.image
        # check if we are in RGB so no conversion needed
        if colorSpace == 'RGB':  # no need for conversion
            return image

        newColorSpace = 'cv2.COLOR_RGB2' + colorSpace
        return cv2.cvtColor(image, eval(newColorSpace))

    # endregion

    # region operator transformations

    def Sobel(self, orientationX, orientationY, kernelSize=3, toSpace='GRAY', toChannel=None):
        """
        Apply the Sobel Transform to any image channel, not just gray
        :param orientationX: the x coordinate of the orientation vector
        :param orientationY: the y coordinate of the orientation vector
        :param kernelSize:
        :param toSpace: from which color space to extract the channel. GRAY is default
        :param toChannel: what channel to extract from the previous color space. Applicable only not in GRAY space
        :return: a np.array with the applied sobel transform on the wanted channel
        """
        if toSpace == 'GRAY':
            channel = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        else:
            channel = self.toChannel(toSpace, toChannel)
        return cv2.Sobel(channel, cv2.CV_64F, orientationX, orientationY, ksize=kernelSize)

    def sideOperator(self, kernelSize=15, toSpace='RGB', toChannel=None):
        if toSpace == 'GRAY':
            channel = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        elif toChannel is None:
            return sideImageFilter(self.toSpace(toSpace), kernelSize)  # return here
        else:
            channel = self.toChannel(toSpace, toChannel)

        return sideChannelFilter(channel, kernelSize)

    # endregion

    # region feature vector extraction

    def toSpatialFeatureVector(self, size=SP_SIZE, image=None):
        """
        Resize the image and return it as a feature vector
        :param size: the size to resize the image
        :param image: None -> use self.image
        :return: the resized image in vector format
        """
        if image is None:
            image = self.image
        features = cv2.resize(image, size).ravel()
        return features

    def hogFeatures(self, image=None, toSpace='RGB', toChannel='R', orientation=9, pix_per_cell=4, cell_per_block=2,
                    vis=False, feature_vec=True):
        if image is None:
            image = self.image
            image = self.toChannel(toSpace, toChannel, image)  # get the desired channel

        if vis is True:
            features, hog_image = hog(image, orientations=orientation, pixels_per_cell=(pix_per_cell, pix_per_cell),
                                      cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                                      visualise=True, feature_vector=False)
            return features, hog_image
        else:
            features = hog(image, orientations=orientation, pixels_per_cell=(pix_per_cell, pix_per_cell),
                           cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                           visualise=False, feature_vector=feature_vec)
            return features

    def generateFeaturesVector(self, histogramsChannels=None, hogChannels=None,
                               spatialSize=None, hist_bins=H_BINS,
                               orient=ORIENT, pix_per_cell=PPC, cell_per_block=CPB, featureVector=True):

        featuresVector = np.array([])

        if spatialSize is not None:
            spatialVector = self.toSpatialFeatureVector(size=spatialSize)
            featuresVector = np.concatenate(featuresVector, spatialVector)

        if histogramsChannels is not None:
            for channel in histogramsChannels:
                pos = channel.rfind('.')
                toSpace = channel[:pos]
                toChannel = channel[pos + 1:]

                his_values, his_features, his_centers = self.channelsHistogram(self.toChannel(toSpace, toChannel),
                                                                               n_bins=hist_bins,
                                                                               )

                featuresVector = np.concatenate([featuresVector, his_features])

        if hogChannels is not None:
            for channel in hogChannels:
                pos = channel.rfind('.')
                toSpace = channel[:pos]
                toChannel = channel[pos + 1:]

                hog_features = self.hogFeatures(toSpace=toSpace, toChannel=toChannel,
                                                orientation=orient, pix_per_cell=pix_per_cell,
                                                cell_per_block=cell_per_block,
                                                vis=False, feature_vec=featureVector)

                if len(featuresVector) == 0:
                    featuresVector = hog_features
                else:
                    featuresVector = np.concatenate([featuresVector, hog_features])

        return featuresVector

    # endregion

    # region histograms

    def channelsHistogram(self, image=None, n_bins=H_BINS):
        # compute the histogram for each channel separately
        if image is None:
            image = self.image
        channels = 1 if len(image.shape) == 2 else 3

        if channels == 3:
            histogram = [np.histogram(image[:, :, i], bins=n_bins) for i in range(channels)]
            histogramPerChannel = (histogram[0][0], histogram[1][0], histogram[2][0])
            histogramFeaturesVector = np.concatenate(histogramPerChannel)
        else:
            histogram = [np.histogram(image, bins=n_bins)]
            histogramPerChannel = (histogram[0][0],)
            histogramFeaturesVector = histogramPerChannel[0]

        histogramEdges = histogram[0][1]
        histogramCenters = (histogramEdges[1:] + histogramEdges[0:len(histogramEdges) - 1]) / 2

        return histogramPerChannel, histogramFeaturesVector, histogramCenters

    @staticmethod
    def histogram(binaryImage, axis=0):
        """
        Gets as input a binary image and outputs the sum per column of 1's
        The image should be single channel with 1's and 0's
        No check is done on the values (for speed) and only for image shape which is expected to be 2
        :param binaryImage: the binary image to create the histogram from
        :param axis: 0 or 1 to create the histogram along X (axis=0) or along Y (axis=1)
        :return: an ndarray with the number of 1's per column (or row if axis=1)
        """
        assert len(binaryImage.shape) == 2, 'only binary images, single channel are allowed'
        imageHalfHeight = int(2 * binaryImage.shape[0] / 4)
        return np.sum(binaryImage[imageHalfHeight:binaryImage.shape[0]], axis=axis)

    @staticmethod
    def removeNoise(histogram):
        """
        Takes a histogram and tries to identify the lane by removing non-sharp regions of data
        :param histogram:
        :return: Returns a new noise free histogram and a list with the peak points
        """
        noiseThreshold = HISTOGRAM_NOISE_THRESHOLD
        laneWidth = LANE_WIDTH_IN_PIXELS  # outside that windows it is considered noise blob and will be removed
        laneHalf = int(laneWidth / 2)
        histogramSize = len(histogram)
        maxPeaks = 4

        histogram[histogram < HISTOGRAM_NOISE_THRESHOLD] = 0
        sortedIndexes = np.argsort(histogram)
        clearedHistogram = np.zeros_like(histogram)
        lanePositions = []

        for index in sortedIndexes[::-1]:
            step = 1
            if histogram[index] == 0:
                continue

            fromIndex = max(index - laneHalf, 0)
            toIndex = min(index + laneHalf, histogramSize)

            clearedHistogram[fromIndex:toIndex] = histogram[fromIndex:toIndex]
            histogram[fromIndex:toIndex] = 0
            lanePositions.append(index)
            if len(lanePositions) == maxPeaks:
                break

        return clearedHistogram, lanePositions

    # endregion

    #   region various functions

    def regionMask(self, nrows=10, ncols=10, rowRange=None, colRange=None):
        rowRange = [0, nrows - 1] if rowRange is None else rowRange
        colRange = [0, ncols - 1] if colRange is None else colRange

        # checking index ranges
        rowRange[1] = min(rowRange[1], nrows - 1)
        rowRange[0] = min(rowRange[0], nrows - 1)
        rowRange[1] = max(rowRange[1], 0)
        rowRange[0] = max(rowRange[0], 0)

        colRange[1] = min(colRange[1], ncols - 1)
        colRange[0] = min(colRange[0], ncols - 1)
        colRange[1] = max(colRange[1], 0)
        colRange[0] = max(colRange[0], 0)

        rowWidth = int(self.image.shape[0] / nrows)
        columnWidth = int(self.image.shape[1] / ncols)

        startY = rowWidth * rowRange[0]
        endY = rowWidth * rowRange[1] + rowWidth
        startX = columnWidth * colRange[0]
        endX = columnWidth * colRange[1] + columnWidth

        mask = np.zeros_like(self.image[:, :, 0])
        mask[startY:endY, startX:endX] = 1
        return mask

    # endregion

    def __del__(self):
        pass


# noinspection PyPep8Naming
class Filters:
    def __init__(self):
        self.filters = {}

    def __getitem__(self, name):
        return self.filters[name]

    def __setitem__(self, key, value):
        self.defineFilter(key, value)

    def getFilter(self, name):
        return self.filters[name]

    def defineFilter(self, name, channelFunction):
        def newFilter(low=0, high=255, *channels):
            return self.binaryFilter(channelFunction, (low, high), *channels)

        self.filters[name] = newFilter

    @staticmethod
    def binaryFilter(channelFunction, thresholds, *channels):
        changedChannel = channelFunction(*channels)
        binaryOutput = np.zeros_like(changedChannel, dtype=np.uint8)
        binaryOutput[(changedChannel >= thresholds[0]) & (changedChannel <= thresholds[1])] = 1
        return binaryOutput

    @staticmethod
    def identity(channel):
        return channel

    @staticmethod
    def L1Channel(channel):
        functionOnChannel = np.absolute(channel)
        return np.uint8(255 * functionOnChannel / np.max(functionOnChannel))

    @staticmethod
    def L2Channel(channelX, channelY):
        functionOnChannel = np.sqrt(channelX ** 2 + channelY ** 2)
        return np.uint8(255 * functionOnChannel / np.max(functionOnChannel))

    @staticmethod
    def gradientDirection(channelX, channelY):
        functionOnChannel = np.arctan2(np.absolute(channelY), np.absolute(channelX))
        return functionOnChannel


# noinspection PyPep8Naming
class PipeLine:
    def __init__(self, i: ImageChannels, f: Filters):
        self.i = i  # image channels
        self.f = f  # filter functions
        self.p = {}  # pipeline dictionary

    def changeImageChannel(self, newChannel: ImageChannels):
        self.i = newChannel

    def changeFilterSet(self, newFilters: Filters):
        self.f = newFilters

    def setPipe(self, name, *pipes, action=None):
        self.p[name] = (pipes, action)

    def run(self, name):
        if name in self.p:
            return self.pipeline(*self.p[name][0], action=self.p[name][1])  # pipes and action

    def pipeline(self, *pipes, action=None):
        assert len(pipes) >= 1, 'at least one pipeline'
        result = []
        for args in pipes:
            assert len(args) >= 4, 'at least 4 arguments, the filter ,the low and high thresholds and the channels'
            filterName = args[0]
            low = args[1]
            high = args[2]
            # this is a little to complicated.
            # self.i[channelName] returns a tuple (function, *arguments)
            # so self.i[channelName][0] gets the function and calls it with the
            channels = [self.i[channelName][0](*self.i[channelName][1:]) for channelName in args[3:]]
            # apply the filter to the defined channels
            result.append(self.f[filterName](low, high, *channels))

        if action is None:
            # if the pipeline is just one action then do not return a list
            # but simple the element of the list
            if len(result) == 1:
                return result[0]
            else:
                return result
        if action == 'AND':
            return self.allOf(*result)
        if action == 'OR':
            return self.anyOf(*result)
        if action == 'SUB' and len(result) >= 2:
            subArrays = result[1:]
            return self.subFrom(result[0], *subArrays)
        if action == 'NOT' and len(result) == 1:
            return np.logical_not(result[0])
        else:
            raise Exception('undefined action')

    @staticmethod
    def allOf(*boolArrays):
        result = boolArrays[0]
        for boolArray in boolArrays:
            result = result & boolArray
        return result

    @staticmethod
    def anyOf(*boolArrays):
        result = boolArrays[0]
        for boolArray in boolArrays:
            result = result | boolArray
        return result

    @staticmethod
    def subFrom(startingArray, *boolArrays):
        for boolArray in boolArrays:
            startingArray &= np.logical_not(startingArray & boolArray)
        return startingArray
