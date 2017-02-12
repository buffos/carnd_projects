from helpers.imagetools import *
from helpers.plotting import *

frame = ImageChannels()
frame.loadImage('./training.data/vehicles/GTI_Left/image0009.png')
# frame.loadImage('./training.data/non-vehicles/GTI/image10.png')

show_images([frame.image])

# single channel
hg, hf, hc = frame.channelsHistogram(frame.toChannel("RGB", 'R'))
plotChannelHistograms(hg, hc)

# single channel
hg, hf, hc = frame.channelsHistogram(frame.toChannel("HLS", 'S'))
plotChannelHistograms(hg, hc)

# from RGB color space histogram
hg, hf, hc = frame.channelsHistogram()  # default is the image stored inside in RGB
plotChannelHistograms(hg, hc, titles=['R histogram', 'G histogram', 'B histogram'])

# from YCrCb color space histogram
image = frame.toSpace('YCrCb')
hg, hf, hc = frame.channelsHistogram(image)
plotChannelHistograms(hg, hc, titles=['Y histogram', 'Cr histogram', 'Cb histogram'])

# hog features extraction
featuresR, hog_imageR = frame.hogFeatures(toSpace='RGB', toChannel='R', orientation=9, pix_per_cell=8, cell_per_block=2,
                                          vis=True, feature_vec=False)
featuresG, hog_imageG = frame.hogFeatures(toSpace='RGB', toChannel='G', orientation=9, pix_per_cell=8, cell_per_block=2,
                                          vis=True, feature_vec=False)
featuresB, hog_imageB = frame.hogFeatures(toSpace='RGB', toChannel='B', orientation=9, pix_per_cell=8, cell_per_block=2,
                                          vis=True, feature_vec=False)
show_images([hog_imageR, hog_imageG, hog_imageB], per_row=3, titles=['R Channel', 'G Channel', 'B Channel'])

featuresY, hog_imageY = frame.hogFeatures(toSpace='YCrCb', toChannel='Y', orientation=9, pix_per_cell=8,
                                          cell_per_block=2, vis=True, feature_vec=False)
featuresCr, hog_imageCr = frame.hogFeatures(toSpace='YCrCb', toChannel='Cr', orientation=9, pix_per_cell=8,
                                            cell_per_block=2, vis=True, feature_vec=False)
featuresCb, hog_imageCb = frame.hogFeatures(toSpace='YCrCb', toChannel='Cb', orientation=9, pix_per_cell=8,
                                            cell_per_block=2, vis=True, feature_vec=False)
show_images([hog_imageY, hog_imageCr, hog_imageCb], per_row=3, titles=['Y Channel', 'Cr Channel', 'Cb Channel'])

fv = frame.generateFeaturesVector(histogramsChannels=['HLS.S', 'HLS.S'],
                                  hogChannels=['YCrCb.Y', 'YCrCb.Cr', 'YCrCb.Cb'])
