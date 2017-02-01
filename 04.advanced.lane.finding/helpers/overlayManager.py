import cv2
import numpy as np

# import imagetools
# import plottingHelpers as plt


# noinspection PyPep8Naming
class OverlayManager:
    def __init__(self):
        self.texts = {}  # entries will be (msg,@x,@y)
        self.polygons = {}  # entries will be (points,color,alpha,border?)
        self.image = None

        self.textColor = (255, 255, 255)
        self.textSize = 0.5
        self.textThickness = 2
        self.textAlpha = 0.9
        self.textGamma = 0.

        self.areaAlpha = 0.6
        self.areaGamma = 0.
        self.borderThickness = 1

    def setImage(self, image):
        self.image = image

    def getImage(self):
        return self.image

    def addText(self, key, value):
        if not isinstance(value, dict):
            raise Exception('Text should be a dictionary with msg,x,y as keys')
        self.texts[key] = value

    def addPolygon(self, key, value):
        if not isinstance(value, dict):
            raise Exception('Polygon should be a dictionary with points,color,border as keys')
        self.polygons[key] = value

    def clearPolygons(self):
        self.polygons = {}

    def clearTexts(self):
        self.texts = {}

    def clearAll(self):
        self.polygons = {}
        self.texts = {}

    def drawPolygons(self):
        areaOverlay = self.image.copy()

        for polygon in self.polygons.values():
            cv2.fillPoly(areaOverlay, [polygon['points'].astype(np.int32)], polygon['color'])
            if polygon['border'] is True:  # plot borders too
                cv2.polylines(areaOverlay, [polygon['points']], True, (0, 0, 0), self.borderThickness)

        self.image = cv2.addWeighted(areaOverlay, self.areaAlpha, self.image, 1 - self.areaAlpha, self.areaGamma)

        return self.image

    def drawTexts(self):
        textOverlay = self.image.copy()

        for text in self.texts.values():
            cv2.putText(textOverlay, text['msg'],
                        (text['x'], text['y']),
                        cv2.FONT_HERSHEY_COMPLEX,
                        self.textSize,
                        self.textColor,
                        self.textThickness)

        self.image = cv2.addWeighted(textOverlay, self.textAlpha, self.image, 1 - self.textAlpha, self.textGamma)

        return self.image
