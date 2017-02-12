from numpy import pi

DEBUG_ON = True

DEGREE = pi / 180.
HISTOGRAM_NOISE_THRESHOLD = 20

LANE_WIDTH_IN_PIXELS = 200  # for noise reduction
LANE_WIDTH_SEARCH_MIN = 60  # while building lanes
LANE_WIDTH_SEARCH_MAX = 200

XinM = 3.5 / 400
YinM = 0.02

# SVC PARAMETERS

PPC = 8  # pixels per cell
CPB = 2  # cells per block
ORIENT = 9  # number of histogram orientations
H_BINS = 32  # histogram Bins
H_RANGE = (0, 256)  # histogram Range
SP_SIZE = (32, 32)  # spatial feature vector size
SAMPLE_SHAPE = (64, 64)  # training samples size
