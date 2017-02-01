from global_variables import LANE_WIDTH_SEARCH_MAX, LANE_WIDTH_SEARCH_MIN, XinM, YinM, DEBUG_ON
from plottingHelpers import plotImages

import logging
import numpy as np
import cv2
from collections import deque

PEAKS_ARE_CLOSE = 60  # pixels
SMALLEST_DISTANCE_BETWEEN_LANES = 200
LARGEST_DISTANCE_BETWEEN_LANES = 400
ALLOWED_R_DIFFERENCE = 0.5  # percent
CURVE_POINTS = 72  # number of points to use to draw the curves
ALLOWED_EQ_DISTANCE = 0.01


# noinspection PyPep8Naming
def equations_L2(eq_1, eq_2):
    """
    L2 difference between the derivative of the 2nd order polynomials
    :param eq_1: 1st 2nd order polynomial equation
    :param eq_2: 2nd 2nd order polynomial equation
    :return: the L2 Distance
    """
    return (4 * (eq_1[0] - eq_2[0]) ** 2 + (eq_1[1] - eq_2[1]) ** 2) ** 0.5


# noinspection PyPep8Naming
def createWeightedEquation(old_equation, new_equation, k):
    """
    Find W such that eq_3 = Weq_1 + (1-W)new_equation
    has distance exactly k from eq_1
    :param old_equation:
    :param new_equation:
    :param k: the required distance
    :return: the appropriate weighted equation
    """
    a, b = old_equation[0], new_equation[0]
    c, d = old_equation[1], new_equation[2]

    # Used http://www.wolframalpha.com/input (mathematica ONLINE) for the Calculation
    r1 = (4 * a ** 2 + 8 * a * b + 4 * b ** 2 + c ** 2 + 2 * c * d + d ** 2)
    if r1 != 0:
        W = (-(k * r1) ** 0.5 + r1) / r1
    else:  # use the old equation
        W = 1
    return W * old_equation + (1 - W) * new_equation


# noinspection PyPep8Naming
class Lanes:
    # region init & setters and getters
    def __init__(self):
        self.logger = logging.getLogger('main_app.Lanes')
        self.logger.setLevel(logging.DEBUG)

        self.detected = False  # was the line detected in the last iteration?
        self.recentPeaks = deque(maxlen=10)  # x peaks of the last n fits of the lanes
        self.recentPolys = deque(maxlen=10)  # the polynomials for each of the lanes for the last n-fits
        self.recentRcurves = deque(maxlen=10)  # latest R curves
        self.drivingLane = 0  # left lane is the default
        self.drivingLanePeaks = None
        self.drivingLanesDetected = 0
        self.carPosition = 0.
        self.distanceFromLanes = []  # distance of vehicle from each line
        self.missingLanes = None  # ie [False, False, False, True] means we have lost the 4th lane
        self.consecutiveTimesWithFewerLanes = 0
        self.isPeakDataOk = False  # no peak data yet stored
        self.isLaneDataOk = None  # for every detected lane if previous step caused any problems
        self.updatePeakDataInSteps = 0  # in how many steps i should update peak data
        self.updatePeakDataEvery = 0  # steps

        self.colors = [(0, 255, 0),
                       (70, 130, 180),
                       (237, 41, 57),
                       (220, 20, 60),
                       (164, 90, 82),
                       (181, 126, 220)
                       ]
        self.stats = {'updatedPeaks': 0,  # times lanePeaksUpdated. # keeping some stats on lane detection
                      'droppedPeaks': 0,
                      'moreLanesReceived': 0,
                      'lessLanesReceived': 0,
                      'changedNoOfLanes': 0,
                      'badLaneSpacing': 0
                      }

    def addNewLanePeaks(self, lanePeaks, image):
        """
        Get a new list containing the possible lane positions.
        Some error handling. If empty , flag to query again next frame
        :param lanePeaks:
        :param image: just to create the ImageHeight and ImageWidth when needed
        :return:
        """
        lanePeaks = sorted(lanePeaks, key=int)  # for some reason i got reversed peaks , so sorting
        lanePeaks = lanePeaks[0:4]
        self.logger.debug("LanePeak Received: {0!s}".format(lanePeaks))

        if not lanePeaks or len(lanePeaks) < 2:
            self.logger.debug("- LanePeak Received == NONE or less than 2")
            self.updatePeakDataInSteps = 0  # update again in the next frame
            self.stats['droppedPeaks'] += 1
            if self.isPeakDataOk is True:  # there is a stored peaked List
                # get the last one and store it to lanePeaks. I should always push data to compare
                # with previous frame
                lanePeaks = self.recentPeaks[-1]
                self.logger.debug("-- LanePeak Retrieved Last: {0!s}".format(lanePeaks))
            else:
                self.logger.debug("-- NO LANE PEAK DATA")
                return None
        else:
            # first do this check if we keep missing a lane for consecutive frames count to 10 and then accept i lost it
            if self.drivingLanesDetected > len(lanePeaks):
                self.consecutiveTimesWithFewerLanes += 1
                self.stats['lessLanesReceived'] += 1
                self.logger.debug(
                    "- LanePeaks less than stored. for x{0!s} Times".format(self.consecutiveTimesWithFewerLanes))
            else:
                self.consecutiveTimesWithFewerLanes = 0

            # if i am storing LanePeaks for the first Time
            if self.isPeakDataOk is False:  # first time storing data
                # TODO: CHECK HERE FOR QUALITY SPACING
                self.drivingLanesDetected = len(lanePeaks)  # the number of possible lanes found
                self.isPeakDataOk = True
                self.updatePeakDataInSteps = self.updatePeakDataEvery  # update peak data again in N frames
                self.logger.debug("- Saving data for the first Time: {0!s}".format(lanePeaks))

            # if the lane peaks returned are less than those i should have recover those
            elif self.drivingLanesDetected > len(lanePeaks):  # the new lane data misses one lane
                # first check if i should give up and set those as new lanes
                self.logger.debug(
                    "- Received LESS lane data. Stored {0!s} got {1!s}".format(self.drivingLanesDetected,
                                                                               len(lanePeaks)))
                if self.consecutiveTimesWithFewerLanes >= 10 and self.isCorrectlySpaced(lanePeaks):
                    # stop searching for more lanes if the data I got make sense (have lane spacing range)
                    self.drivingLanesDetected = len(lanePeaks)
                    self.updatePeakDataInSteps = self.updatePeakDataEvery
                    self.consecutiveTimesWithFewerLanes = 0
                    self.stats['changedNoOfLanes'] += 1
                    self.logger.debug("-- NEW number of Lanes: {0!s}".format(self.drivingLanesDetected))
                else:
                    lanePeaks, success = self.mergeLanes(lanePeaks)  # bring back the missing data from previous
                    if success is True:
                        self.updatePeakDataInSteps = 0  # attempt to get correct data failed so try again
                        self.logger.debug("--- SUCCESS Recovered the missing lane")
                    else:  # could not recover so dropping the lane
                        self.logger.debug("--- FAILED Did not recovered the missing lane")
                        self.drivingLanesDetected = len(lanePeaks)
                        self.updatePeakDataInSteps = self.updatePeakDataEvery  # now wait for N steps to find again
                    self.stats['droppedPeaks'] += 1

                    self.logger.debug("--- Corrected Lane Data {0!s}".format(lanePeaks))

            # if i get More lanes than the previous steps
            elif self.drivingLanesDetected < len(lanePeaks):  # i got more peak data
                self.logger.debug(
                    "- Received MORE lane data. Stored {0!s} got {1!s}".format(self.drivingLanesDetected,
                                                                               len(lanePeaks)))
                self.stats['moreLanesReceived'] += 1
                lanePeaks, success = self.handleNewLaneDetected(lanePeaks)
                if success is True:  # the newly add lane is actually a valid one
                    self.drivingLanesDetected = len(lanePeaks)
                    self.updatePeakDataInSteps = self.updatePeakDataEvery  # no need to update
                    self.stats['changedNoOfLanes'] += 1
                    self.logger.debug(
                        "-- found a NEW valid lane. Increasing Lanes to {0!s}".format(self.drivingLanesDetected))
                else:  # we got back a previous LanePeaks set
                    self.updatePeakDataInSteps = 0  # attempt to get correct data failed so try again
                    self.stats['droppedPeaks'] += 1
                    self.logger.debug("-- the NEW lane was invalid. Dropping and requesting data next frame")
            else:
                # if i reach her then 1. Its not first frame and i have the same number of lanes
                if self.isCorrectlySpaced(lanePeaks):
                    self.updatePeakDataInSteps = self.updatePeakDataEvery  # will not update for 10 frames
                    self.logger.debug("- Lanes are correctly spaced. Updating again in {0!s} steps".format(
                        self.updatePeakDataInSteps))
                else:
                    lanePeaks = self.recentPeaks[-1]
                    self.updatePeakDataInSteps = 0  # try again at next frame
                    self.stats['droppedPeaks'] += 1
                    self.logger.debug("- Lanes were NOT correctly spaced. Getting previous data")

        self.stats['updatedPeaks'] += 1
        self.logger.debug("- UPDATING LANE DATA")

        # identify driving lane peaks and check they are not dropped in the new peaks.
        # if they are restore them
        # Driving lane can be shifted but not dropped.
        self.calculateCarPositionAndDrivingLane(image)
        if self.drivingLanePeaks is None and len(lanePeaks) >= 2:
            self.drivingLanePeaks = [lanePeaks[self.drivingLane], lanePeaks[self.drivingLane + 1]]
        else:
            lanePeaks, hasChanged = self.handleMissingDrivingLane(self.drivingLanePeaks, lanePeaks)
            if hasChanged is True:
                self.logger.debug('---- RESTORED DRIVING LANE')

        self.logger.debug("--- DRIVING LANE PICKS: {0!s}".format(self.drivingLanePeaks))

        oldLanePeaks = self.recentPeaks[-1] if self.recentPeaks and len(self.recentPeaks) > 0 else 'NONE'
        lanePeaks = lanePeaks[0:4]

        self.logger.debug("--- OLD LANE DATA: {0!s}".format(oldLanePeaks))
        self.logger.debug("--- NEW LANE DATA: {0!s}".format(lanePeaks))
        self.recentPeaks.append(lanePeaks)

    def getPreviousLanePeak(self, step=-1):
        if self.recentPeaks:
            return self.recentPeaks[step]
        else:
            return None

    def getPreviousEquations(self, step=-1):
        if self.recentPolys:
            return self.recentPolys[step]
        else:
            return None

    def getPreviousRcurves(self, step=-1):
        if self.recentRcurves:
            return self.recentRcurves[step]
        else:
            return None

    # endregion

    # region Calculate curvature and car position
    def calculateRCurves(self, mask, latestEquations=None):
        if latestEquations is None:
            latestEquations = self.getPreviousEquations()

        if latestEquations is None: # still None
            return None

        maskHeight = mask.shape[0]
        curvatures = []

        for z in latestEquations:
            curvatures.append(self.calculateRCurve(z, maskHeight))
        return curvatures

    def calculateCarPositionAndDrivingLane(self, unwrappedImage):
        """
        Calculate 1. The Driving lane, 2. The distance of the car from the center of the driving lan
        3. The distance from the other calculated lanes
        :param unwrappedImage:
        :return:
        """
        columnStart = 0
        lanePeaks = self.getPreviousLanePeak()
        if lanePeaks is None:  # nothing stored yet in Lane Data
            return None
        imageHeight = unwrappedImage.shape[0] - 1
        while unwrappedImage[imageHeight, columnStart, 0] == 0:
            columnStart += 1
        columnEnd = unwrappedImage.shape[1] - 1
        while unwrappedImage[imageHeight, columnEnd, 0] == 0:
            columnEnd -= 1

        screenMiddle = (columnStart + columnEnd) / 2

        lanesMiddle = [(lanePeaks[i] + lanePeaks[i + 1]) / 2 for i in range(0, len(lanePeaks) - 1)]
        carFromLanes = [abs(screenMiddle - middle) * XinM for middle in lanesMiddle]
        if len(carFromLanes) > 0:
            carLanePosition = carFromLanes.index(min(carFromLanes))
            carPosition = (screenMiddle - lanesMiddle[carLanePosition]) * XinM  # to get the sign of the distance
        else:
            carLanePosition = 0
            carPosition = 0

        self.drivingLane = carLanePosition
        self.carPosition = carPosition
        self.distanceFromLanes = carFromLanes

        return carPosition

    @staticmethod
    def calculateRCurve(coefficients, atY):
        if coefficients is None:
            return None
        atY = atY * YinM
        a = coefficients[0]
        b = coefficients[1]
        return (1 + (2 * a * atY + b) ** 2) ** 1.5 / np.absolute(2 * a)

    # endregion

    def calculatePolyLineEquations(self, mask):
        """
        from the latest Lane Peaks we calculate the laneEquations
        :param mask: this is just needed to get the image height and width. any correctly sized image will do
        :return: a list with the lane Equation coefficients per lane
        """
        self.logger.debug("- CALCULATE POLYLINE EQUATIONS")
        if self.isPeakDataOk is False:  # no data to work
            self.logger.debug("-- NO LANE PEAK DATA")
            return None
        latestPeaks = self.getPreviousLanePeak(-1)
        maskHeight = mask.shape[0]
        self.logger.debug("-- LanePeaks : {0!s}".format(latestPeaks))
        self.logger.debug("-- Image Height : {0!s}".format(maskHeight))

        laneEquations_MAX = []
        laneEquations_SELECTED = []
        previousEquations = self.getPreviousEquations()
        previousRcurves = self.getPreviousRcurves()

        self.logger.debug("-- Previous Equations : {0!s}".format(previousEquations))
        self.logger.debug("-- Previous Rcurves : {0!s}".format(previousRcurves))

        for index, peak in enumerate(latestPeaks):
            # searching to regions. A wide and a small. If the wide fails to get quality results i will
            # fall back to the small window
            z_MAX = self.createPolyFitData(mask, peak, LANE_WIDTH_SEARCH_MAX)
            self.logger.debug("---- Lane :{0!s} MAX Equation: {1!s}".format(index, z_MAX))

            if (z_MAX is None) and (previousEquations is not None and index < len(previousEquations) and (
                        previousEquations[index] is not None)):
                laneEquations_MAX.append(previousEquations[index])  # append previous corresponding equation
            else:
                laneEquations_MAX.append(z_MAX)

        self.logger.debug("---- Lane Equations MAX: {0!s}".format(laneEquations_MAX))

        self.logger.debug("---- STARTING QUALITY CONTROL")
        # QUALITY CONTROL 1: Check that 1st order derivatives are close
        # in the L2 sense. If not create a weighted new equation
        if previousEquations is not None and len(previousEquations) > 0:
            for index, equation in enumerate(laneEquations_MAX):

                if index < len(previousEquations):
                    equation_old = previousEquations[index]
                else:  # i have more lanes this time than the previous frame and no correspondence
                    equation_old = equation  # cheating here and just accepting the new Radius

                equationDistance = equations_L2(equation, equation_old)
                self.logger.debug("------ Eq_Max Distance {0:>5} for Lane {1!s}".format(equationDistance, index))

                if equationDistance <= ALLOWED_EQ_DISTANCE:
                    laneEquations_SELECTED.append(equation)
                    self.logger.debug("------ Eq_Max Selected for Lane {0!s}".format(index))

                else:  # will create the equation for the MIN SEARCH WINDOW
                    self.logger.debug("------ Lane :{0!s} Calculating MIN Equation for peak {1!s}"
                                      .format(index, latestPeaks[index]))

                    z_MIN = self.createPolyFitData(mask, latestPeaks[index], LANE_WIDTH_SEARCH_MIN)
                    z_MIN = previousEquations[index] if z_MIN is None else z_MIN  # just make sure its not none

                    self.logger.debug("------ Lane :{0!s} Calculated MIN Equation: {1!s}".format(index, z_MIN))

                    equationDistance = equations_L2(z_MIN, equation_old)
                    self.logger.debug("------ Eq_Min Distance {0:>5} for Lane {1!s}".format(equationDistance, index))

                    if equationDistance <= ALLOWED_EQ_DISTANCE:
                        laneEquations_SELECTED.append(z_MIN)
                        self.logger.debug("------ Eq_Min Selected for Lane {0!s}".format(index))
                    else:  # both fail creating weighted equation or previous one
                        weightedEquation = createWeightedEquation(previousEquations[index], z_MIN, ALLOWED_EQ_DISTANCE)
                        laneEquations_SELECTED.append(weightedEquation)

                        self.logger.debug(
                            "------ Lane :{0!s} Calculated Weighted Equation: {1!s}".format(index, weightedEquation))

        else:
            laneEquations_SELECTED = laneEquations_MAX

        # QUALITY CONTROL 2: Check equations within them have not big curvature changes
        # "building" our lanes based on the assumption
        # that the left lane was correctly identified
        R_SELECTED = self.calculateRCurves(mask, laneEquations_SELECTED)

        for index in range(1, len(R_SELECTED)):
            if abs(R_SELECTED[index] - R_SELECTED[index - 1]) / R_SELECTED[index - 1] < ALLOWED_R_DIFFERENCE:
                pass
            elif R_SELECTED[index] < 10000:
                # Big change in R curve so copying the lane on the left and adjusting the c-coefficient
                # so it peaks at the correct point
                # which is just done by adding to the 3rd coefficient the lanePeaks difference
                laneEquations_SELECTED[index] = laneEquations_SELECTED[index - 1].copy()  # copy the lane to the left
                lanePeakDifference = (latestPeaks[index] - latestPeaks[index - 1])  # * XinM
                laneEquations_SELECTED[index][2] = laneEquations_SELECTED[index - 1][2] + lanePeakDifference * XinM
                self.logger.debug("------ R curve Control: Copied lane {0!s} to Lane {1!s}".format(index - 1, index))

        self.logger.debug("---- FINISHED QUALITY CONTROL")

        self.recentPolys.append(laneEquations_SELECTED)  # add the latest equations
        self.recentRcurves.append(self.calculateRCurves(mask, latestEquations=laneEquations_SELECTED))
        self.logger.debug("---- CALCULATED EQUATIONS FOR {0!s} LANES".format(len(laneEquations_SELECTED)))
        self.logger.debug("---- THE NEW EQUATIONS ARE: {0!s}".format(laneEquations_SELECTED))
        self.logger.debug("- CALCULATE POLYLINE EQUATIONS FINISHED")
        return laneEquations_SELECTED

    # region Plotting functions
    def plotLatestPolyLines(self, mask, step=-1):
        """
        Create a histogram plot of the latest (if step=-1) lanes
        :param mask:
        :param step:
        :return:
        """
        import matplotlib.pyplot as plt

        plt.xlim(0, mask.shape[1])
        plt.ylim(0, mask.shape[0])

        if self.recentPolys is not None:
            latestEquations = self.recentPolys[step]
        else:
            raise Exception('plotLatestPolyLines failed: No recent equations stored to plot')

        maskHeight = mask.shape[0]
        plotStep = maskHeight / CURVE_POINTS

        for z in latestEquations:
            l = np.polyval(z, np.arange(0, maskHeight + plotStep, plotStep) * YinM)
            plt.plot(l[::-1] / XinM, np.arange(0, maskHeight + plotStep, plotStep), color='green', linewidth=3)
        plt.show()

    def plotLanesOverImage(self, road, mask, debug=True):
        latestEquations = self.getPreviousEquations()
        if latestEquations is None:
            return np.zeros_like(road.uframe.image).astype(np.uint8), np.zeros_like(road.uframe.image).astype(np.uint8)
            # raise Exception('plotLanesOverImage failed: No recent equations stored to plot')

        self.logger.debug("-PLOTTING STARTED ")
        self.logger.debug("--Plotting {0!s} Areas".format(len(latestEquations) - 1))

        maskHeight = mask.shape[0]
        maskWidth = mask.shape[1]
        plotStep = maskHeight / CURVE_POINTS
        # road.overlay.image = np.dstack((mask * 255, mask * 255, mask * 255))
        road.overlay.image = np.zeros_like(road.uframe.image).astype(np.uint8)

        Xs = [np.polyval(z, np.arange(0, maskHeight + plotStep, plotStep) * YinM) / XinM for z in latestEquations]

        for i in range(1, len(Xs)):
            Xs[i][(Xs[i] - Xs[i - 1]) <= 0] = Xs[i - 1][(Xs[i] - Xs[i - 1]) <= 0]  # no overlap
            Ys = np.arange(0, maskHeight + plotStep, plotStep)
            pts1 = np.vstack((Xs[i - 1], Ys)).T
            pts2 = np.vstack((Xs[i][::-1], Ys[::-1])).T
            pts = np.vstack((pts1, pts2))

            road.overlay.addPolygon('lane' + str(i - 1),
                                    {'points': pts, 'color': self.colors[i - 1], 'border': 0})  # (points,color,border?)

        unwrappedAnnotated = road.overlay.drawPolygons()
        wrappedAnnotated = cv2.addWeighted(road.frame.image, 1, road.cam.toProjected(unwrappedAnnotated), 0.6, 0)
        if debug is True:
            plotImages(unwrappedAnnotated, wrappedAnnotated)
        return wrappedAnnotated, unwrappedAnnotated

    def plotTextOverImage(self, road, wrappedImageToDrawOn, currentFrameNumber):
        r_curves = self.getPreviousRcurves()
        wrappedImageHeight = wrappedImageToDrawOn.shape[0]
        wrappedImageWidth = wrappedImageToDrawOn.shape[1]
        road.overlay.image = np.zeros_like(road.uframe.image).astype(np.uint8)  # set the image to draw over

        if r_curves is None:
            return road.overlay.image

        if currentFrameNumber % 10 == 0 or currentFrameNumber == 1:
            road.overlay.clearTexts()
            for i in range(0, len(r_curves)):
                road.overlay.addText("R" + str(i),
                                     {"msg": "R{0}: {1:>10.2f}m".format(i + 1, r_curves[i]), "x": 50,
                                      "y": 40 * (i + 1)})

            road.overlay.addText("carPosition",
                                 {"msg": "Distance from Lane Center: {0:>6.2f}m".format(self.carPosition),
                                  "x": int(wrappedImageWidth / 2) - 200, "y": 50})

            road.overlay.addText("drivingLane", {"msg": "Driving Lane: {0}".format(self.drivingLane + 1),
                                                 "x": int(wrappedImageWidth / 2) - 200, "y": 80})

            road.overlay.addText("drivingLanesDetected",
                                 {"msg": "Driving Lanes Detected: {0}".format(self.drivingLanesDetected),
                                  "x": int(wrappedImageWidth / 2) - 200, "y": 110})

            road.overlay.addText("updatedLanePosition",
                                 {"msg": "Updated Lane Position: {0} times".format(self.stats["updatedPeaks"]),
                                  "x": wrappedImageWidth - 400, "y": 50})

            road.overlay.addText("droppedLanePosition",
                                 {"msg": "Dropped Received Lane Data: {0} times".format(self.stats["droppedPeaks"]),
                                  "x": wrappedImageWidth - 400, "y": 80})

            road.overlay.addText("moreLanesReceived",
                                 {"msg": "Received More Lane Data: {0} times".format(self.stats["moreLanesReceived"]),
                                  "x": wrappedImageWidth - 400, "y": 110})

            road.overlay.addText("lessLanesReceived",
                                 {"msg": "Received Less Lane Data: {0} times".format(self.stats["lessLanesReceived"]),
                                  "x": wrappedImageWidth - 400, "y": 140})

            road.overlay.addText("changedNoOfLanes",
                                 {"msg": "Changed No of Lanes tracked: {0} times".format(
                                     self.stats["changedNoOfLanes"]),
                                     "x": wrappedImageWidth - 400, "y": 170})

        wrappedTexts = cv2.addWeighted(wrappedImageToDrawOn, 1, road.overlay.drawTexts(), 0.9, 0)

        # if True:
        #     plotImages(wrappedTexts, wrappedImageToDrawOn)
        return wrappedTexts

        # endregion

    @staticmethod
    def plotSlidingWindows(binaryMask, equation, nonZeroX, nonZeroY, columnIndices, slidingWindows):
        import matplotlib.pyplot as plt
        fitY = np.linspace(0, binaryMask.shape[0] - 1, binaryMask.shape[0])
        fitX = equation[0] * fitY ** 2 + equation[1] * fitY + equation[2]

        outImage = np.dstack((binaryMask * 255, binaryMask * 255, binaryMask * 255))
        outImage[nonZeroY[columnIndices], nonZeroX[columnIndices]] = [255, 0, 0]

        for window in slidingWindows:
            cv2.rectangle(outImage, window[0], window[1], (0, 255, 0), 2)

        plt.imshow(outImage)
        plt.plot(fitX / XinM, fitY / YinM, color='blue', linewidth=2.0)
        plt.xlim(0, binaryMask.shape[1])
        plt.ylim(binaryMask.shape[0], 0)
        plt.show()

    # region Error Handling
    def errorEmptyPeakData(self):
        if self.isPeakDataOk is False:
            return None
        else:
            self.updatePeakDataInSteps -= 1  # one step closer to update peak data
            return self.recentPeaks[-1]

    def mergeLanes(self, newLanePeaks):
        oldLanePeaks = self.recentPeaks[-1]
        success = False  # successfully recovered the missing lane

        self.logger.debug('---- MERGING LANES')
        self.logger.debug('------- OLD LANES {0!s}'.format(oldLanePeaks))
        self.logger.debug('------- NEW LANES {0!s}'.format(newLanePeaks))

        mergedLanes = []

        counter_old = 0
        counter_new = 0

        while counter_old <= len(oldLanePeaks) and counter_new <= len(newLanePeaks):
            if counter_old == len(oldLanePeaks):  # completed oldlanePeaks
                mergedLanes += newLanePeaks[counter_new:]  # add the rest
                counter_old = len(oldLanePeaks) + 1
                counter_new = len(newLanePeaks) + 1
            elif counter_new == len(newLanePeaks):
                mergedLanes += oldLanePeaks[counter_old:]
                counter_old = len(oldLanePeaks) + 1
                counter_new = len(newLanePeaks) + 1
            elif abs(oldLanePeaks[counter_old] - newLanePeaks[counter_new]) <= PEAKS_ARE_CLOSE:
                mergedLanes.append(newLanePeaks[counter_new])
                counter_new += 1
                counter_old += 1
            elif oldLanePeaks[counter_old] < newLanePeaks[counter_new]:
                mergedLanes.append(oldLanePeaks[counter_old])
                counter_old += 1
            else:
                mergedLanes.append(newLanePeaks[counter_new])
                counter_new += 1

        for i in range(1, len(mergedLanes)):
            if (mergedLanes[i] - mergedLanes[i - 1]) > LARGEST_DISTANCE_BETWEEN_LANES:
                self.logger.debug('------- MERGED PARTIALLY LANES {0!s}'.format(mergedLanes[0:i - 1]))
                self.logger.debug('------- FROM {0!s} to {1!s}'.format(0, i - 1))
                return mergedLanes[0:i - 1], True

        self.logger.debug('------- MERGED LANES {0!s}'.format(mergedLanes))
        return mergedLanes, True

    def handleNewLaneDetected(self, lanePeaks):
        if self.isCorrectlySpaced(lanePeaks):  # everything is fine return data
            return lanePeaks, True  # success we have a new added lane
        else:
            return self.recentPeaks[-1], False  # we have noise. Throw it away and get the previous one.

            # endregion

    def handleMissingDrivingLane(self, oldDrivingLane, lanePeaks):
        leftLine = oldDrivingLane[0]
        rightLine = oldDrivingLane[1]

        changedData = False  # if it changes to true i will update some Lanes variables

        # check if leftLine is in lanePeaks
        for i in range(0, len(lanePeaks)):
            if abs(leftLine - lanePeaks[i]) < PEAKS_ARE_CLOSE:  # found left line
                leftLine = lanePeaks[i]  # the new position of the left driving lane
                if i == len(lanePeaks) - 1:  # Left Line is the last line in lanePeaks
                    lanePeaks.append(rightLine)
                    changedData = True
                    break
                elif abs(rightLine - lanePeaks[i + 1]) < PEAKS_ARE_CLOSE:  # the right line exist
                    rightLine = lanePeaks[i + 1]  # the new position of the right driving lane
                    break
                else:  # the right line is missing. Adding it
                    lanePeaks.insert(i + 1, rightLine)
                    changedData = True
                    break
            elif leftLine < lanePeaks[0]:  # left of the new left peak and not close
                changedData = True
                lanePeaks.insert(0, leftLine)
                if abs(rightLine - lanePeaks[1]) < PEAKS_ARE_CLOSE:
                    rightLine = lanePeaks[1]
                    break
                else:
                    lanePeaks.insert(1, rightLine)
                    break
            elif leftLine > lanePeaks[i]:
                # passed the peaks close to LeftLine and did not find any match.
                # and since previous step was not close, i am inserting left line at previous step
                lanePeaks.insert(i, leftLine)  # add at position i and push the other elements to the right
                changedData = True
                # now check for right lane
                if abs(rightLine - lanePeaks[i + 1]) < PEAKS_ARE_CLOSE:  # right line is in place
                    rightLine = lanePeaks[i + 1]  # the new position of the right driving lane
                    break
                else:
                    lanePeaks.insert(i + 1, rightLine)  # inserted the right line too
                    break

        if changedData is True:
            self.updatePeakDataInSteps = 0
            self.drivingLanesDetected = len(lanePeaks)
            self.drivingLanePeaks = [leftLine, rightLine]  # with their new updated positions
        return lanePeaks, changedData

    def createPolyFitData2(self, binaryMask, laneColumn, searchWidth=LANE_WIDTH_SEARCH_MAX):
        halfLane = int(searchWidth / 2)

        xPoints = []
        yPoints = []

        for r in np.arange(0, binaryMask.shape[0])[::-1]:  # for all rows from bottom of the image to top
            row_Weight = 0
            nonZeroCount = 0

            # calculate those for every row, since weight can shift the center of the line
            fromColumn = max(laneColumn - halfLane, 0)
            toColumn = min(laneColumn + halfLane, binaryMask.shape[1])

            for c in np.arange(fromColumn, toColumn):  # for the columns in the search region
                row_Weight += (c - laneColumn) * binaryMask[r, c]
                nonZeroCount += binaryMask[r, c]

            if nonZeroCount != 0:
                laneColumn += int(row_Weight / nonZeroCount)
                xPoints.append(laneColumn)
                yPoints.append(r)

        xPoints = np.array(xPoints).astype(np.int16) * XinM
        yPoints = np.array(yPoints).astype(np.int16) * YinM

        # self.logger.debug("POLYFIT - xPOINT {0!s}".format(xPoints))
        # self.logger.debug("POLYFIT - yPOINT {0!s}".format(yPoints))

        if xPoints.size <= 3 or yPoints.size == 3:
            return None

        try:
            equation = np.polyfit(yPoints, xPoints, deg=2)
        except Exception as e:
            self.logger.error("ERROR CREATING EQUATION {0!s".format(e))
            self.logger.error("X-POINTS {0!s}  Y-POINTS {1!s}".format(xPoints, yPoints))
        else:
            return equation

    def createPolyFitData(self, binaryMask, laneColumn, searchWidth=LANE_WIDTH_SEARCH_MAX, debug=DEBUG_ON):
        halfLane = int(searchWidth / 2)
        slidingWindows = 9
        windowHeight = np.int(binaryMask.shape[0] / slidingWindows)
        nonZeroIndices = binaryMask.nonzero()
        nonZeroIndicesY = np.array(nonZeroIndices[0])
        nonZeroIndicesX = np.array(nonZeroIndices[1])

        minimumActivePixels = 50

        laneIndices = []
        slidingWindowsList = []

        for window in range(slidingWindows):
            repeat = True
            while repeat is True:
                windowTop = binaryMask.shape[0] - window * windowHeight
                windowBottom = binaryMask.shape[0] - (window + 1) * windowHeight
                windowLeft = laneColumn - halfLane
                windowRight = laneColumn + halfLane

                activeIndices = ((nonZeroIndicesY >= windowBottom) & (nonZeroIndicesY <= windowTop) & (
                    nonZeroIndicesX >= windowLeft) & (nonZeroIndicesX <= windowRight)).nonzero()[0]
                laneIndices.append(activeIndices)

                if len(activeIndices) > minimumActivePixels:
                    newLaneColumn = np.int(np.mean(nonZeroIndicesX[activeIndices]))
                    columnDisplacement = abs(newLaneColumn - laneColumn)
                    laneColumn = newLaneColumn
                    if columnDisplacement < 10:
                        repeat = False
                        slidingWindowsList.append([(windowLeft, windowBottom), (windowRight, windowTop)])
                else:
                    repeat = False
                    slidingWindowsList.append([(windowLeft, windowBottom), (windowRight, windowTop)])

        laneIndices = np.concatenate(laneIndices)

        xPoints = nonZeroIndicesX[laneIndices] * XinM
        yPoints = nonZeroIndicesY[laneIndices] * YinM

        if xPoints.size <= 2 or yPoints.size == 2:
            return None

        try:
            equation = np.polyfit(yPoints, xPoints, deg=2)
            if debug is True:
                self.plotSlidingWindows(binaryMask, equation, nonZeroIndicesX, nonZeroIndicesY, laneIndices,
                                        slidingWindowsList)
        except Exception as e:
            self.logger.error("ERROR CREATING EQUATION {0!s".format(e))
            self.logger.error("X-POINTS {0!s}  Y-POINTS {1!s}".format(xPoints, yPoints))
        else:
            # self.logger.debug("EQUATION {0!s}".format(equation))
            return equation

    def isCorrectlySpaced(self, someLanePeaks):
        global SMALLEST_DISTANCE_BETWEEN_LANES, LARGEST_DISTANCE_BETWEEN_LANES
        for i in range(0, len(someLanePeaks) - 1):
            laneDistance = someLanePeaks[i + 1] - someLanePeaks[i]  # forward distance
            # if not withing distance limits
            if not (SMALLEST_DISTANCE_BETWEEN_LANES < laneDistance < LARGEST_DISTANCE_BETWEEN_LANES):
                self.stats['badLaneSpacing'] += 1
                return False
        return True
