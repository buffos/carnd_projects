**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./submission_images/undistort_output.png "Undistorted"
[image2a]: ./submission_images/test1.jpg "Road Transformed"
[image2b]: ./submission_images/test1_undistorted.jpg "Road Transformed undistored output"
[side_operator]: ./submission_images/my_side_operator.png "Experimental Side Operator"
[image3a]: ./submission_images/challenge_video_14.jpg "Hard Challenge Video Frame"
[image3b]: ./submission_images/challenge_video_14_binary_mask.png "Hard Challenge Video Frame Binary Example"
[image4]: ./submission_images/source_destination_points.jpg "Source and destination Points"
[image5a]: ./submission_images/sliding_windows.jpg "Sliding Windows Line 1"
[image5b]: ./submission_images/sliding_windows1.jpg "Sliding Windows Line 2"
[image6]: ./submission_images/sliding_windows_drawn_area.jpg "Output"



## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.


---

### Writeup / README


#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

You can submit your writeup as markdown or pdf.
[Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.

You're reading it!

### Camera Calibration

I have created a **camera class** that handles all camera related functions. Its in the helpers folder. `camera.py`.

The calibration from images takes place in the `Camera.calibrateFromImages` function, which takes as parameters a glob pattern ,
the expected shape of chessboard corners (rows, columns) and if i want to monitor the process (showing images) or not. 

The procedure *mirrors* the one in the lectures


* I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world.
Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.
Thus, `objectGrid` is just a replicated array of coordinates, and `object_points` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.
  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.

* `Camera.calibrateFromImages` then stores those points in the class (you can later call 
again the function with another glob pattern and continue to append points (grid - image)


* I then used the stored output `self.object_points` and `self.image_points` to compute 
the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function which is done inside the function 
`Camera.calibrate(imageSize)`. Both cameraMatrix and distortionCoeefficients are stored inside the class inside a dictionary with 
key being the imageSize they have been computed for. So if `cam = Camera()` then cam.cameraMatrix[imageSize] will give the
camera matrix stored for that image size (if any). The same for distortionCoeefficients.

* Then I created two functions to undistort images. One from a file and one from in memory images.
They work as follows. They load the image (or get it as an argument) and check if the image size has cameraMatrix and distortionCoeefficients
stored inside the class. If not, the run the calibrate function to obtain those. If there are already stored matrices, they just fetch them
from the dictionary. Then it simply applies the `cv2.undistort` function obdaining the result

* I then can save the calibration to a pickle file, so there is no need to do the procedure everytime I run the code.

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

I will use one of the test_images provided. The code that does this job is as simple as

```
     cam.loadCalibration('./calibration.p')
     cam.undistortImageFromFile('./test_images/test1.jpg',save=True)
```

It will load the calibration file for the camera and then undistort the provided image

---

![alt text][image2a]

---

to get the following distortion free image


![alt text][image2b]


---

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.
I have put a lot of work in this field to make it easy for me to test various patterns. I have created 3 classes (all found in the helpers folder in imagetools.py)

* an **ImageChannels()** : which helps me easily extract channel information from an image. The class can be easily used as follows.
For example  `frame = ImageChannels()`

    * frame.defineChannel('HSV.V', 'toChannel', 'HSV', 'V')  does  exactly what you expect to. It will defines a partial function and stores it in a dictionary
    with key='HSV.V' and when applied will convert the image to HSV space and give the V channel.
    * to get the channel you simple say `frame.getChannel('HSV.V')`
    * I have defined multiple channels which i could use, including one that returns a channel after a Sobel operator has been applied and also
    * a custom operator I created that removes color casting and expands lanes to make them more visible. They problem is that it is too slow to be used
    at a video but the results were interesting. 
    You can view an example of its results in the following image and the code is at `helpers/experimentalSideOperator.py`


    ![alt text][side_operator]



    * To use it inside the ImageChannels class you can simply `frame.defineChannel('Preconditioner', 'sideOperator', 31, 'RGB')` 
    which will define it under the key=Preconditioner and use it by applying operators from size 3 to size 31 (this is why its slow) to the RGB color space
    but you can even be more specific and apply it to just the  B channel by adding another parameter `frame.defineChannel('Preconditioner', 'sideOperator', 31, 'RGB', 'B')`
    * So this class produces image channels
* an **Filters() class** which takes a channel(s) as input and applies a function on those. I have created just 4 functions inside that class but
one can create as many as he likes. The 4 functions are
    * an L1Channel that takes the channel, calculates its absolute value and then normalizes the output
    * an L2Channel that takes two channels and calculates the L2 Norm and then again normalizes the output 
    * a Gradient Function, that takes two channels and outputs arc of the tangent
    * an Identity function that just let the channel pass unaltered
    * I stored the filters in a dictionary with a friendly name like
        ```
        filters['absoluteFilter'] = filters.L1Channel
        filters['magnitudeFilter'] = filters.L2Channel
        filters['directionGradientFilter'] = filters.gradientDirection
        filters['identity'] = filters.identity 
        ```

* a **third class**, that uses the above two called **PipeLine()**, that uses a Filter class and an ImageChannels class. Its better to explain what it does
with a simple code snippet

    ```
        pipe.setPipe('lightColorsMask',
                    ('identity', 0, 180, 'HSV.H'),
                    ('identity', 0, 50, 'HSV.S'),
                    ('identity', 200, 255, 'HSV.V'),
                    action='AND'
                    ) 
    ```

    It does the following. It creates a pipeline, it names it `lightColorMask` and this pipeLine has 3 actions


        * One that gets the H channel from the image associated with the pipe, passes it through the Filter with the name identity
        , which in our case is a pass function, and then thresholds the value allowing values from 0 to 180
        * One that gets the S channels from the HSV space, and thresholds values from 0 to 50
        * One that gets the V channel from the HSV space and thresholds values from 200 to 255
        * and then an instruction to AND the results from the previous actions (each one gave a binary mask )
        * allowed actions are AND, OR and SUB

    * to **run** the pipeline you just invoke `pipe.run("the name of the user defined action")`

This way , it was very easy to experiment with numerous pipelines. For example in the following image taken from the hard challenge video

![alt text][image3a]

I used the following PipeLine

```
    pipe.setPipe('WhiteMask',
                ('identity', 220, 255, 'RGB.R'),
                ('identity', 220, 255, 'RGB.G'),
                ('identity', 220, 255, 'RGB.B'),
                action='AND'
                )

    pipe.setPipe('YellowMask',
                ('identity', 10, 40, 'HSV.H'),
                ('identity', 30, 250, 'HSV.S'),
                ('identity', 200, 255, 'RGB.R'),
                ('identity', 200, 255, 'RGB.G'),
                action='AND'
                )

and then simply

    mask = pipe.run("WhiteMask") | pipe.run("YellowMask") 

    created the following mask

```

![alt text][image3b]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

I expanded my Camera class to add this functionality. It is simply a wrapper around the `cv2.getPerspectiveTransform` that calculates
the perspective Transformation matrix and it requires 4 points in a plane as `source points`  and their corresponding coordinates
`destination points` in the final transform. 

You can find it in the Camera Class. I use 3 functions

    * camera.getDefaultPerspectiveCorrectionPoints() which simply return the src and destination points
    * camera.calculatePerspectiveMatrix(src,dst) that calculate the cameraPerspective Matrix 
    * camera.initializeProjectionPoints(image), that basically calls the above two and its called when the camera is initialized

I chose the hardcode the source and destination points in the following manner:

```
        src = np.array([[220. / 1280. * imageWidth, 719. / 720. * imageHeight],
                        [1220. / 1280. * imageWidth, 719. / 720. * imageHeight],
                        [750. / 1280. * imageWidth, 480. / 720. * imageHeight],
                        [550. / 1280. * imageWidth, 480. / 720. * imageHeight]], np.float32)

        dst = np.array([[x_offset + 0. * imageWidth, 1 * imageHeight],
                        [x_offset + 1 / lanes * imageWidth, 1 * imageHeight],
                        [x_offset + 1 / lanes * imageWidth, 0. * imageHeight],
                        [x_offset + 0. * imageWidth, 0. * imageHeight]], np.float32)

```

As you see, except from the image Width and height which are parameters i choose to be able to squeeze the X axis in the final Transfromation
so I could handle multiple lanes. Also x_offset, handles how much to the left i can view from the transformation.

For  x_offset = 50 and lanes = 4 and an Image of (720, 1280)  resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 220, 720      | 50, 720      | 
| 1120, 720     | 370, 720      |
| 740, 480      | 370, 0        |
| 550, 480      | 50, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

First of all I choose to detect all lanes (if possible) in the image, so that made the process a lot more complicated. 
The basic steps are:
 * Create a binary mask of the image, hopefully containing only the lanes
 * Create a 'histogram' by summing by column (image height) the number of active pixels per column
 * The peaks are an indication of a possible lane
 * In contrast to just considering 2 lines and stretching out the birds eye view, I cannot take for granded that i have one left from the middle line
 and one from the right. So to find the possible lanes  (you can view the code in helpers/imagetools.py ... removeNoise function)
    * I sort the histogram
    * take the first value, keep it, consider that a lane has at least LANE_WIDTH_IN_PIXELS (which is a user defined constant) and zero everything
    in a range of center that value and radius LANE_WIDTH_IN_PIXELS/2
    * Then take the 2nd best value that **exist now* , which after the previous step is not near this one.
    * In this fashion I get at most 4 candidate lines to explore
* Then , with the candidate positions of the lines at hand, I follow the sliding window technique as described in the lectures. 

Since managing Lanes is quite complex , i created a separate class `helpers\lanesManager` that had to deal with the following tasks

* Bookkeeping Lanes. That consist of what to do when
    * Receive no lane data
    * Receive less lane points than before
    * Receive more points that before
    * Receive points that are not correctly spaced to be considered lanes
    * Define a driving lane (the one that the car is closest too) and keep track of it and restore if not in the data received
    * Decide when to drop a lane or added to the video annotation
* Calculate the equations for each of the lanes , using the binary mask and the Lane Peaks that were identified by analysis of the histogram
    * Calculation was mostly done inside `helpers\lanesManager  createPolyFitData(self, binaryMask, laneColumn, searchWidth=LANE_WIDTH_SEARCH_MAX, debug=DEBUG_ON):`
    where I give the mask, the Lane position to examine, how wide the search should be (width of the bounding box) and if I want to view intermediate results (images)
    * Calculation was done by using the sliding window approach as shown in the lectures. We start from the base of the Lane Peak, as provided by the histogram
    analysis, create small bounding boxes, splitting the image height in N parts, add the active pixels in a pool, calculate the "mean" of the active pixels,
    so if we find that there are more on a direction (left or right), in the next step as we go up, we shift the bounding box to that direction. In the end , we use the pool of active pixels
    we have accumulated as we went up the image, and use numpy's polyfit function to create a second order polynomial to fit those pixels
    * After that the laneManager does some quality analysis, to see how different is this equation from the previous steps equation.
    I do that by comparing the first order derivatives of the polynomials, by requiring the [a,b] coefficients (the derivative
    of the ay^2+b^y+c is ay+b and i take those a and b and form a vector), seen as vectors, are close in the L2 norm sense.
    * If they are found not to be within a user defined threshold then I create a weighted average such that, the new equation is exactly at the required threshold.
    You can view this function at `helpers\lanesManager  createWeightedEquation(old_equation, new_equation, k)`, where k is the threshold that has to be achieved
    * I also tried analysis of Radius of curvatures between lanes, but that has proven unsuccessful
* Plot over images the calculated lanes and areas they create
* Calculate the position of the car
* Keep track of Bookkeeping statistics


You can view an example of the sliding window approach in the following image (taken from the hard challenge video)

![alt text][image5a]
![alt text][image5b]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

This is done in the class laneManager `helpers\lanesManager function: calculateRCurve` by simply implementing the formula given in the lectures

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

To do all that overLay plotting i created another class `helpers\overlayManager` that is more less a thin wrapper around the cv2
plotting functionality , making it easier to use in the LanesManager Class. There are 2 functions I use for plotting over images in that class

* lanesManager.plotLanesOverImage
* lanesManager. plotTextOverImage

 Here is an example of the result produced by the previous images (the sliding window example). You can see both the plotted area in the
 bird's eye view and then projected back to the initial image
![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

I have uploaded my videos to youtube and here are the results of all 3 videos (the harder challenge is a failure but just for reference)
I can upload them in Github, if that is necessary, but I thought I could save space

* Here's a [link to my project video result](https://youtu.be/LOrw1w_fyrU)
* Here's a [link to the challenge video](https://youtu.be/NBxxAsAmfTE)
* And a [failed attempt to the hard challenge](https://youtu.be/YmKr-Vcgt90)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

I think this is a big subject and the harder challenge video shows the problems

* It is not possible to create "one filter to rule them all". Lighting conditions alter the perception of colors, but an extra sensor can help automate this procedure
* In rural environments , it might be impossible to identify the right lane , because it blends in the environment and you cannot infer it from the one
that is easier to identify (in the middle) because they have very different shapes in sharp turns
* In the problem we haven't considered the change in perspective when riding uphill (or down), or we bump over obstacles
* What happens when we change Lanes
* What happens in a road full of trafic (white cars should be banned!!! )
* I am pretty sure that my lane's Manager Bookeeping is far from perfect and i

I think that this approach is only viable in urban settings.

#### 2. Summary of how my code is structured

* There is a roadManager class, that contains 2 imageChannels (one for the original perspective and one
for the bird's view), the filter class, the pipeline, a camera and the lanes manager

* The road class receives the video frame, uses the camera to undistort and create the bird's eye view
* The stored that view in a channel and applies a pipeline.
* Calculates the histogram removes any noise and send it to the lanes Manager
* The Lanes Manager extracts the new lines, calculates the equations, does the bookkeeping and then plots
the calculated lanes areas over to blank image which is merged with the original frame.

This modularity has helped me to experiment a lot.
Extensive logging is used throughout the code, using the logging python module.
Debugging is turned on and off by a global variable found in the `helpers\global_variables.py` file.

