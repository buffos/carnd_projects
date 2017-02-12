from helpers.cars_detector import CarDetector

from collections import deque
# from math import sqrt
import logging

HOG_VECTORS = 3
WEIGHT_POSITION = 0.1
WEIGHT_DIMENSIONS = 0.1
DROP_AFTER_N_FRAMES = 20
UPDATE_CONFIDENCE_TAG_EVERY = 10
ALIVE_THRESHOLD = 10  # if it is detected for more than those frames THEN display it


# noinspection PyPep8Naming
def boundingBoxArea(box):
    """
    Area of a rectangulara x1,y1,x2,y2
    :param box:
    :return:
    """
    return (box[2] - box[0] + 1) * (box[3] - box[1] + 1)


# noinspection PyPep8Naming
def overlapPercent(box1, box2):
    """
    Returns the overlap area percentage between 2 bounding boxes
    box1 is the reference box
    :param box1:
    :param box2:
    :return:
    """
    xx2 = min(box1[2], box2[2])
    xx1 = max(box1[0], box2[0])
    yy2 = min(box1[3], box2[3])
    yy1 = max(box1[1], box2[1])
    w = max(0, xx2 - xx1 + 1)
    h = max(0, yy2 - yy1 + 1)
    areaBox1 = boundingBoxArea(box1)
    areaBox2 = boundingBoxArea(box2)
    overlap = max(w * h / areaBox1, w * h / areaBox2)
    return overlap


# noinspection PyPep8Naming
class Car:
    def __init__(self):
        self.id = None
        self.box = None
        self.centroid = None
        self.speed = None
        self.confidence = None
        self.displayedConfidence = None  # update this every N frames to stop jitter
        self.aliveForFrames = 0
        self.lastCentroid = deque([], 5)  # centroid previous position
        self.updatedInFrame = 0

    def __str__(self):
        msg = '\n\nCar ID: {}\n'.format(self.id)
        msg += 'Bounding Box x1= {} y1 = {} x2 = {} y2 = {}\n'.format(*self.box)

        msg += "Centroid: X {} Y {} \n".format(*self.centroid)
        msg += "Previous Centroid: {} \n".format(self.lastCentroid)

        msg += 'Confidence: {} %\n'.format(self.confidence)
        msg += 'Tracked for: {} frames\n'.format(self.aliveForFrames)
        msg += 'Last Updated in: {} th frame\n'.format(self.updatedInFrame)
        return msg

    def setBox(self, box):
        """
        Update the bounding box of the car based on the previous positions of it
        For this implementation I only use the previous position of the box
        :param box:
        :return:
        """
        if self.box is None:
            self.box = box
            self.centroid = [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]
            self.lastCentroid.append(self.centroid)
        else:
            # Creating a weighted update
            new_centroid = [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]
            old_centroid = self.centroid

            received_height = box[3] - box[1]
            received_width = box[2] - box[0]

            new_centroid[0] = new_centroid[0] * WEIGHT_POSITION + old_centroid[0] * (1 - WEIGHT_POSITION)
            new_centroid[1] = new_centroid[1] * WEIGHT_POSITION + old_centroid[1] * (1 - WEIGHT_POSITION)

            new_height = received_height * WEIGHT_DIMENSIONS + (self.box[3] - self.box[1]) * (1 - WEIGHT_DIMENSIONS)
            new_width = received_width * WEIGHT_DIMENSIONS + (self.box[2] - self.box[0]) * (1 - WEIGHT_DIMENSIONS)

            # calculating the new bounding box
            self.box[0] = int(new_centroid[0] - new_width / 2)
            self.box[2] = int(new_centroid[0] + new_width / 2)
            self.box[1] = int(new_centroid[1] - new_height / 2)
            self.box[3] = int(new_centroid[1] + new_height / 2)

            self.centroid = new_centroid
            self.lastCentroid.append(new_centroid)


# noinspection PyPep8Naming
class CarTracker:
    def __init__(self):
        self.logger = logging.getLogger('main_app.carTracker')
        self.logger.setLevel(logging.DEBUG)
        self.cd = CarDetector()
        self.middlePoint = None
        self.image = None
        self.cars = []
        self.currentFrame = 0
        self.car_detector = CarDetector()

    def updateFrame(self, image):
        """
        It takes an image (part of video) , identifies the cars
         and tracks them
        :param image:
        :return:
        """
        self.currentFrame += 1
        self.image = image.copy()

        detected = self.car_detector.detectCars(image)
        picks = self.car_detector.non_max_suppression_fast(detected, 0.2)

        self.logger.debug(" CURRENT CAR LIST\n")
        self.printCars()

        self.logger.debug("\nNew Picks {0!s}\n".format(picks))

        self.addCars(picks)
        self.removeOldCars()
        if len(self.cars) == 0:
            self.logger.debug("EMPTY.... HELP")
        # self.printCars()
        return self.drawCars()

    def addCars(self, picks):
        """
        Based on identified bounding boxes (regions) of cars
        it tries to add them to the tracker.
        :param picks: bounding box, confidence score
        :return:
        """
        for pick in picks:
            self.addCar(pick[0], pick[1])

    def addCar(self, b_box, confidence):
        """
        Adding or Updating a car that is located in a bounding box
        :param b_box:
        :param confidence:
        :return:
        """
        car = Car()
        car.id = len(self.cars) + 1
        car.setBox(b_box)
        car.confidence = confidence
        car.displayedConfidence = confidence
        car.aliveForFrames += 1
        car.updatedInFrame = self.currentFrame

        self.logger.debug("Processing BoundingBox {} {} {} {}".format(*b_box))
        self.logger.debug("Trying to add a NEW car {0!s}".format(car))

        if len(self.cars) == 0:
            # i wont be merging the car with an existing one
            # so just adding the last positions to the deque
            self.cars.append(car)
            return True
        else:
            self.logger.debug("Car List is NOT empty. Trying to find a match")
            possible_matches = self.possibleMatches(newCar=car)
            if len(possible_matches) > 0:
                self.updateCar(possible_matches[0], car)
                return False
            self.logger.debug('ADDED a NEW car with ID == {}'.format(car.id))
            self.cars.append(car)  # car was not in collection adding it
            return True

    def printCars(self):
        """
        Prints a list of the cars, with all the details. For debugging mainly
        """
        for car in self.cars:
            self.logger.debug(car)

    def removeOldCars(self):
        """
        If a car has not been updated for many frames, remove it from the tracker
        :return:
        """
        self.cars = [car for car in self.cars if (self.currentFrame - car.updatedInFrame) < DROP_AFTER_N_FRAMES]
        for i, car in enumerate(self.cars):  # update id's
            car.id = i + 1

    def drawCars(self):
        """
        Draw the bounding boxes + various messages on the image of the video
        :return:
        """
        for car in self.cars:
            if car.aliveForFrames >= ALIVE_THRESHOLD:
                msg = 'ID: {0:>2}\n'.format(car.id)
                msg += 'conf:{0:.2f}%\n'.format(car.displayedConfidence * 100)
                msg += 'active: {} frames'.format(car.aliveForFrames - car.aliveForFrames % 5)
                self.car_detector.draw_boxes(self.image, tuple(car.box), msg)

        return self.image

    def possibleMatches(self, newCar: Car):
        """
        Returns a list of cars that are possible matches for the car that is to be added to the tracker
        if they are close enough
        :param newCar:
        :return:
        """
        selectedCars = []
        for car in self.cars:
            if self.carsAreClose(car, newCar):
                selectedCars.append(car)

        return selectedCars

    def updateCar(self, car1: Car, from_car2: Car):
        """
        update the stored car, with the information of the new car created from the new bounding box
        :param car1:
        :param from_car2:
        :return:
        """
        self.logger.debug('Updating Car {}\n'.format(car1.id))

        car1.setBox(from_car2.box)  # or weighted solution
        car1.aliveForFrames += 1
        car1.confidence = from_car2.confidence
        car1.updatedInFrame = from_car2.updatedInFrame
        if car1.aliveForFrames % UPDATE_CONFIDENCE_TAG_EVERY == 0:
            car1.displayedConfidence = from_car2.confidence

    @staticmethod
    def carsAreClose(car1: Car, car2: Car):
        """
        Cars are close if their bounding boxes overlap some percent
        :param car1:
        :param car2:
        :return:
        """
        op = overlapPercent(car1.box, car2.box)
        if op > 0.3:
            return True
        else:
            return False
