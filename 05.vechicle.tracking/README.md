**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.


[//]: # (Image References)
[image1]: ./output_images/car_not_car.png
[image2a]: ./output_images/vechicle_image_feature_vectors.jpg
[image2b]: ./output_images/non_vechicle_image_feature_vectors.jpg
[image3]: ./output_images/bounding_boxes.png
[image4]:  ./output_images/bounding_boxes_merged.png
[video1]: https://youtu.be/gZcPh29K-9Q?list=PLFiZkfjLt9lI1QMqb_BO2dTGf4oeV6k5J

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  
---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### A. Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

1.  I extended my ImageTools class (`imagetools.py`), which I created for the previous project (project 4) adding functionality to:
    * Calculate a Histogram of any channel in any color space (`channelsHistogram` member function)
    * Extract the image as a feature vector in any size (`toSpatialFeatureVector` member function)
    * To calculate the HOG features from any channel in every color space (`hogFeatures` member function)
    * A function to easily combine the above in one feature vector (`generateFeaturesVector` member function)

2. The second part was identifying which information to use and which not and extracting those from the relevant training images.

    *   I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

        ![alt text][image1]

    * I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`). 
    I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like.
    I also played alot with histograms.
    Here is an example using the `RGB` color space and HOG parameters of `orientations=8`, `pixels_per_cell=(8, 8)` and `cells_per_block=(2, 2)`:

        ![Vechicle Features][image2a]
            
        ![Non Vehicle Features][image2b]

#### 2. Explain how you settled on your final choice of HOG parameters.

After creating a pipeline to extract feature vectors easily the next phase was just cycling between 

* training the SVM 
* applying classification to test images
* observing the percent of wrong classifications (false positives or false negatives)
* peeking a different set of features to train on.

#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

Everything is contained in `train.svm.py` file. The process is like this:

* Extract features from car and non car images (settled on Histogram of Saturation channel and 
HOG features of every channel of the YCrCb space)
* Normalized data using the StandardScaler
* Split data to train and test 
* Used LinearSVC to train an svc
* Calibrated the results as advised in [this link](https://jmetzen.github.io/2015-04-14/calibration.html)
The basic idea is that various methods tend to have a biased estimation. What you expect when your model says
that probability is 80% of being correct is that that among the samples to which it gave a predict_proba
value close to 0.8, approximately 80% actually belong to the positive class. This is certainly NOT true for the 
non calibrated svm. A calibrated SVM helps a lot in filtering the results

### B. Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

All my car detection code is in the `cars_detector.py`. The code for sliding windows is the `slidingWindowInStripe()` method.
It is written in such a way that i can define stripes (regions) of the image to search using a customized windows size and customized overlap.
The final decision on the region was based on logic - experimentation and compromise in speed

As you can see in the `detectCars` method that uses the above function to detect the cars I used 3 regions (which are defined in the init of the class)
* large: `{'ys': (400, 600), 'xs': (400, 1280), 'size': 96, 'overlap': 0.7}`
* medium: `{'ys': (400, 600), 'xs': (0, 1280), 'size': 128, 'overlap': 0.9}`
* small: `{'ys': (300, 500), 'xs': (400, 1280), 'size': 64, 'overlap': 0.6}`

I could relax even more the sliding windows to get a faster pipeline, accepting some false positives but increasing the speed dramatically.
I also calculate the HOG features for the whole image and subclass it for the cells (the hard part is to do the coordinate transformations correctly).
This gives a speed boost but it is still slow if you explode the number of windows. Also the histogram feature vector has to be calculated per image.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on two scales using YCrCb 3-channel HOG features plus a histogram of saturation in the feature vector, which provided a nice result.  
I also filter boxes based on their confidence score returned from the calibrated classifier. That gives me very few false positives.
Also, as noted above, I calculate HOG features on bigger regions and the extract the corresponding information for each search window.

Here is an example of output I get , before applying any kind of filtering and merging on the bounding boxes:

![alt text][image3]

#### 3. Merging Boxes.

To merge bounding boxes, I did not use heatmaps, because although I liked a lot the history properties inherent to the idea, they can not be vectorized
and the final result is much slower than my implementation

I use non maximal suppression and as a base I used the code found [here.](http://www.pyimagesearch.com/2014/11/17/non-maximum-suppression-object-detection-python/)
My code can be found in `non_max_suppression_fast`  method in the `cars_detector.py` file.
The main changes I made to the code are
* Instead of picking the window with the largest score or the lowest y value as in the original code I merge overlapping boxes
* I also reject boxes that have only K overlaps. K is a parameter. In my final implementation I set K = 2. Setting K=3 actually returns zero
false positives but misses some correct ones. This happens as the car goes deeper into the horizon, since only small boxes (64,64) can detect it.
The problem is that to make things faster I have my overlap parameter for small boxes set to 0.6 to speed things up. Getting that value up, will give more positive hits
and solving all the problems but making the solution much slower. K = 2 was a good compromise

Here is the result of my bounding boxes merging applied in the above image

![alt text][image4]
---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result][video1]


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

My code had for static images the following filters

1. **Confidence Threshold** set to 0.8
2. **Voting Threshold** set to 2 (two bounding boxes have to overlap to accept the bounding box)

The following gave few (4-5) frames of false positives in the whole video. Not much. Increasing Voting to 3, made those disappear but also
, as I explained before some of the car detection in the horizon had problems

To solve that I added another filtering condition. To show a tracked object in the video , it had to be tracked for M frames
All my car tracking code is in the `car_tracker.py` file. Its pretty straightforward.

1. Get the bounding boxes from the cars_detector
2. Check if there is already a tracked object in the region (by comparing the bounding boxes overlapping area percent)
3. If there is an object update, the bounding box is updated (based on a weighted solution heavily weighting on previous steps to avoid excessive flickering)
4. If there is no object in the vicinity of the detected object, just add the new Object.
5. I also track which frame I updated the object and how many frames its "alive" (aka tracked)
6. If I have not updated the object for L frames I remove it from tracking

Some experimentation with the parameters, showed me that I can safely drop the object after 20 frames (less than a second) and not show it
before its tracked for 10 frames.

That way I have no false positives and reliably track my objects. It is actually like having a heatmap for the whole bounding box 

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

1. Obviously my pipeline will fail when my classifier fails. This is with objects it has not been trained on, or with irregular shape.
For example a very long car, a truck , will not be identified.

2. My second concern is speed. Any algorithm that is not fast enough is useless. To improve speed I am willing to sacrifice some false
positives for random frames. Besides safety is failing to identifying cars and falling on them and not aesthetic reasons.
To improve speed , I would first update every N frames and not every frame. I would also decrease my confidence percent to get more identified objects
Probably some false positives that I would filter out as I do now. I guess that if I accept some false positives for random frames (less than 1% of the total number of frames)
I could increase speed to 20x or more

3. Although I really liked the process and learned a lot from the project, my intuition says that I would have much better (and faster) results
if I did the classification in the bird's eye view. The reasons are
    * In the bird's eye view I would not detect car - notCar , but road not_road. That way, i could identify **any** obstacle in the road
    and not just cars, but rocks, animals etc since all those would be classified as "not road"
    * Its easier to track and predict the trajectory of the objects, so fewer frames to track, easier to predict and calculate speed for example
    * Also fewer HOG features would be required to identify a road block from a non road one. So even faster results.
