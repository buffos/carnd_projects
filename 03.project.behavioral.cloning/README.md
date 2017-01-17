## Using Deep Learning to Clone Driving Behavior

![OUTPUT](output.jpg)

### Contents
1. [Project Goals](#goals)
2. [Data Collection](#data-collection)
3. [Data Preprocessing](#data-preprocessing)
4. [Neural Network Architecture](#neural-network-model)
5. [Problem HyperParameters](#problem-hyperparameters)
6. [Training](#training)
7. [Driving Simulations](#driving-simulations)
8. [Running the Simulator](#running-the-simulator)


## Goals

The goal of this project is to train a neural network to drive a car by cloning human behavior. This is done by associating what you see
and what actions you take based on the input. Input in this problem are images and output the steering angle that will be fed to the vehicle.

* Data collection is done by driving around a track , and capturing images (3 cameras are mounted in the car) and steering angles,
which are all logged to a file
* We can record as many data as we like from that single track

The goal is considered accomplished if the neural network manages to complete the track without crashing, or exhibiting any behavior that would be
 dangerous to the human passengers


## Data Collection:

Here I mainly used:

* Data provided by Udacity (driving the car through the center of the track)
* A reverse drive through of the track
* Extra drives of track 1

Driving was done using a PS4 controller

## Data Preprocessing

This is the key part of the project and the goal is to make the data as unbiased as possible

* To keep the data balanced and not biased in zero based angles, since they dominate every dataset, I separated the initial dataset of
 (image, angle) pairs into 3 bags.
  * Left steering angles
  * Almost Zero steering angles
  * Right steering angles

  of course this induces a new **hyperparameter** ,I named it **segregation angle**, and it is the
  threshold that separates a zero steering angle from a *normal* steering angle
* Another important decision is , since the zero-angle steering data are much more if we decide to include them all or
a **fraction** of the data. This is also a **hyperparameter** that one has to experiment with
* In addition to the above steps, some "preconditioning" was done to the angles. Namely some "random" noise was added
to all angles during training time. We do this because a fixed steering angle is not the correct and only solution for the problem
and that change will break the pattern
* If no 'noise' is added then the problem seems more than a classification than a regression one, with classes such
steer left, steer right , no action and
* Using right and left camera data was also explored. I ended up adding those data but the angle adjustment was not optimal



        The optimal solution would be to add those images by using the correct physics which in this case
        means we would have to assume equal angular velocities. That would give us that:


        ω1= v * sinΘ1/r1 and ω2= v * sinΘ2/r2 and since ω1 = ω2 it would give us that
        sinθ1 * r2  = sinθ2 * r1 => arcsin( sinθ1 * r2 / r1)

        We know θ1 we can calculate r1  since ω = Δθ/ Δt = v * sinΘ1/r1 and then r2 = r1 + camera distance
        which we unfortunatelly do not know.


Data are provided to the CNN in batches which are created in-place within a generator. The generator has to fill each time an array of batch_size
which is user defined. It works as follows


The generator gets 3 index arrays, one for each "bag" of data (left, center ,right angles) and to fill the batch data bag it
* Flips a coin to select a bag and gets an image
* Flips a coin to choose if the image is going to be flipped or not, to generate more data
* We add random noise to the recorded angle
* Crops the sky and the car parts of the image and resizes the image to fit the shape of the input of the CNN
* We randomly alter the brightness channel of the image. This way we remove the preference of the network to brighter parts of the track.
If we do not do that, because we train in the first track which is mostly sunny, when we try to run the second track it will try to follow the brighter parts of
the track, thus crashing

## Neural Network Model

This was a straightforward decision. I used the model provided by [NVIDIA](https://arxiv.org/pdf/1604.07316v1.pdf)


Implementation of the NVIDIA paper network

    Layer (type)                     Output Shape          Param #     Connected to
    ====================================================================================================
    Normalization (Lambda)           (None, 66, 220, 3)    0           lambda_input_1[0][0]
    ____________________________________________________________________________________________________
    conv1 (Convolution2D)            (None, 31, 108, 24)   1824        Normalization[0][0]
    ____________________________________________________________________________________________________
    elu_1 (ELU)                      (None, 31, 108, 24)   0           conv1[0][0]
    ____________________________________________________________________________________________________
    conv2 (Convolution2D)            (None, 14, 52, 36)    21636       elu_1[0][0]
    ____________________________________________________________________________________________________
    elu_2 (ELU)                      (None, 14, 52, 36)    0           conv2[0][0]
    ____________________________________________________________________________________________________
    conv3 (Convolution2D)            (None, 5, 24, 48)     43248       elu_2[0][0]
    ____________________________________________________________________________________________________
    elu_3 (ELU)                      (None, 5, 24, 48)     0           conv3[0][0]
    ____________________________________________________________________________________________________
    conv4 (Convolution2D)            (None, 3, 22, 64)     27712       elu_3[0][0]
    ____________________________________________________________________________________________________
    elu_4 (ELU)                      (None, 3, 22, 64)     0           conv4[0][0]
    ____________________________________________________________________________________________________
    conv5 (Convolution2D)            (None, 1, 20, 64)     36928       elu_4[0][0]
    ____________________________________________________________________________________________________
    flatten1 (Flatten)               (None, 1280)          0           conv5[0][0]
    ____________________________________________________________________________________________________
    elu_5 (ELU)                      (None, 1280)          0           flatten1[0][0]
    ____________________________________________________________________________________________________
    dense1 (Dense)                   (None, 1164)          1491084     elu_5[0][0]
    ____________________________________________________________________________________________________
    elu_6 (ELU)                      (None, 1164)          0           dense1[0][0]
    ____________________________________________________________________________________________________
    dense2 (Dense)                   (None, 100)           116500      elu_6[0][0]
    ____________________________________________________________________________________________________
    elu_7 (ELU)                      (None, 100)           0           dense2[0][0]
    ____________________________________________________________________________________________________
    dense3 (Dense)                   (None, 50)            5050        elu_7[0][0]
    ____________________________________________________________________________________________________
    elu_8 (ELU)                      (None, 50)            0           dense3[0][0]
    ____________________________________________________________________________________________________
    dense4 (Dense)                   (None, 10)            510         elu_8[0][0]
    ____________________________________________________________________________________________________
    elu_9 (ELU)                      (None, 10)            0           dense4[0][0]
    ____________________________________________________________________________________________________
    dense5 (Dense)                   (None, 1)             11          elu_9[0][0]
    ====================================================================================================
    Total params: 1,744,503
    Trainable params: 1,744,503
    Non-trainable params: 0


## Problem HyperParameters.

The parameters explored were (besides the usual neural network parameters)

* Segregation angle (which angle will split  almost zero angles from actual steering angles)
* Fraction of zero angles to include while training (this actually plays the list role)
* The amount of noise i should add to a steering angle
* Augmentation angle (for left and right images , since as explained above the correct one can not be calculated)
This actually is pretty critical if one decided to include those in training

## Training

Training was done on GPU (GTX 1080). Almost all possible combination where tried and my final choice for the hyperparameters were

* ANGLE_NOISE = 0.15  # percent
* AUGMENTATION_ANGLE = 0.60
* ITERATIONS_PER_EPOCH = 40000
* FRACTION_OF_ZERO_ANGLES = 1.0
* SEGREGATION_ANGLE = 0.05
* ITERATIONS_PER_EPOCH = 40000
* EPOCHS = 20
* TRAIN_BATCH_SIZE = 128
* VALIDATION_BATCH_SIZE = 250
* LEARNING_RATE = 1e-4

To avoid overfitting I tried using, DropOut layers and Regularization. The best result with the smoother driving behavior was achieved when using
L2 Regularization which was my final choice.

Also the initial training data were split (85%-15%) to **training** and **validation** set and the loss on the validation set was calculated at the end
of each epoch. With L2 Regularization the loss on the validation set was lowered in almost every single step.

## Running the simulator

I have slightly altered the **drive.py** and you need to provide **only the path without the json filename**.
That way if you provide no arguments it will search for a **model.json** in the current path.
To run the current model, just run drive.py without any arguments