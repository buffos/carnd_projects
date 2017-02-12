import logging
from helpers.global_variables import DEBUG_ON
from helpers.car_tracker import CarTracker
from moviepy.editor import VideoFileClip

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


FRAME_COUNTER = 0

import cv2
def process_image(image):
    global car_tracker, FRAME_COUNTER, numberOfFrames
    return car_tracker.updateFrame(image)


# car_detector = CarDetector()
car_tracker = CarTracker()
input_file = 'project_video.mp4'
# input_file = 'test_video.mp4'
output_file = 'project_video_annotated.mp4'
video_clip = VideoFileClip(input_file)
logger.info('processing video...')
numberOfFrames = video_clip.fps*video_clip.duration
new_video_clip = video_clip.fl_image(process_image)
new_video_clip.write_videofile(output_file, audio=False)