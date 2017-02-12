from helpers.imagetools import *

import glob
import time
import numpy as np

from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from sklearn.svm import LinearSVC
from sklearn.calibration import CalibratedClassifierCV
from sklearn.externals import joblib  # it claims to be faster the pickle

if __name__ == '__main__':
    cars = glob.glob('./training.data/vehicles/**/*.png')
    notCars = glob.glob('./training.data/non-vehicles/**/*.png')

    print("Found {} car images".format(len(cars)))
    print("Found {} non-car images".format(len(notCars)))

    frame = ImageChannels()
    carFeatures = []
    nonCarFeatures = []

    for i, car in enumerate(cars):
        percent = i / len(cars) * 100
        print("\rExtracting from car images...{0:5.2f}%".format(percent), end='')
        frame.loadImage(car)
        fv = frame.generateFeaturesVector(histogramsChannels=['HLS.S'],
                                          hogChannels=['YCrCb.Y', 'YCrCb.Cr', 'YCrCb.Cb'])
        carFeatures.append(fv)

    print('')

    for i, non_car in enumerate(notCars):
        percent = i / len(notCars) * 100
        print("\rExtracting from non car images...{0:5.2f}%".format(percent), end='')
        frame.loadImage(non_car)
        fv = frame.generateFeaturesVector(histogramsChannels=['HLS.S'],
                                          hogChannels=['YCrCb.Y', 'YCrCb.Cr', 'YCrCb.Cb'])
        nonCarFeatures.append(fv)

    # normalize features
    X = np.vstack((carFeatures, nonCarFeatures)).astype(np.float64)
    X_scaler = StandardScaler().fit(X)
    X_scaled = X_scaler.transform(X)

    # labels
    y = np.hstack((np.ones(len(carFeatures)), np.zeros(len(nonCarFeatures))))

    # data splitting
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=1)

    # creating the SCV
    svc = LinearSVC(C=1.0)

    # calibrating probabilities
    # https://jmetzen.github.io/2015-04-14/calibration.html
    svc_calibrated = CalibratedClassifierCV(svc)

    # training
    t = time.time()
    print("\nTraining Linear Support Vector Machine Classifier")
    svc_calibrated.fit(X_train, y_train)
    print("Training finished in {0:5.2f}".format(time.time() - t))
    print("Training Accuracy = {0:5.2f}%".format(svc_calibrated.score(X_train, y_train)))
    print("Test Accuracy = {0:5.2f}%".format(svc_calibrated.score(X_test, y_test)))

    # sample predictions
    prediction = svc_calibrated.predict(X_test[0].reshape(1, -1))
    probabilities = svc_calibrated.predict_proba(X_test[0].reshape(1, -1))
    print("Prediction {0}".format(prediction))
    print("Probabilities {0}".format(probabilities))

    # saving
    print("Saving models....")
    joblib.dump(svc, './models/svc.p')
    joblib.dump(svc_calibrated, './models/svc_calibrated.p')
    joblib.dump(X_scaler, './models/scv_data_scaler.p')
    print("Saving completed")
