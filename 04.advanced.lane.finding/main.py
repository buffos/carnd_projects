import logging
from roadManager import RoadManager
from moviepy.editor import VideoFileClip
import os
from global_variables import DEBUG_ON
import plottingHelpers as plt

FRAME_COUNTER = 0


logger = logging.getLogger('main_app')

# create file handler
fh = logging.FileHandler('advanced_lane_logging.log')

if DEBUG_ON is True:
    logger.setLevel(logging.DEBUG)
    fh.setLevel(logging.DEBUG)
else:
    logger.setLevel(logging.INFO)
    fh.setLevel(logging.INFO)

# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)  # ERROR should be in the normal app.
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(fh)
logger.addHandler(ch)


# noinspection PyPep8Naming
def process_image(image):
    global road, FRAME_COUNTER

    FRAME_COUNTER += 1
    logger.debug("Frame No: {:0}".format(FRAME_COUNTER))
    mask = road.processFrame(image=image)
    # plt.showImages(mask,title="n") # DEBUG STATEMENTS

    logger.debug("Updating Lane Picks in {0!s} steps".format(road.lanes.updatePeakDataInSteps))

    if road.lanes.updatePeakDataInSteps == 0:  # its time to update
        newHistogram, lanePeaks = road.identifyLanes(mask)  # search the mask, create a histogram and identify the peaks
        # plt.plotData(newHistogram)  # DEBUG STATEMENTS
        road.lanes.addNewLanePeaks(lanePeaks, road.uframe.image)  # the unwrapped image is expected to be passed
    if road.lanes.updatePeakDataInSteps > 0:
        road.lanes.updatePeakDataInSteps -= 1
    else:
        road.lanes.updatePeakDataInSteps = 0

    road.lanes.calculatePolyLineEquations(mask)
    road.overlay.clearPolygons()  # clears all overLay data from previous step
    wrappedAnnotated, unwrappedAnnotated = road.lanes.plotLanesOverImage(road, mask, debug=DEBUG_ON)

    wrappedAnnotated = road.lanes.plotTextOverImage(road, wrappedAnnotated, FRAME_COUNTER)

    return wrappedAnnotated


camera_file = './calibration.p'

input_file = 'project_video.mp4'
output_file = 'project_video_annotated.mp4'

# input_file = 'challenge_video.mp4'
# output_file = 'challenge_video_annotated.mp4'

# input_file = 'harder_challenge_video.mp4'
# output_file = 'harder_challenge_video_annotated.mp4'

road = RoadManager()
if not os.path.isfile(camera_file):
    logger.info('Calibrating Camera')
    road.cam.calibrateFromImages('./camera_cal/calibration*.jpg', (9, 6), showResults=False)
    road.cam.saveCalibration(camera_file)

road.initData(calibrationFilename=camera_file)
logger.info('processing video...')

video_clip = VideoFileClip(input_file)
new_video_clip = video_clip.fl_image(process_image)
new_video_clip.write_videofile(output_file, audio=False)

