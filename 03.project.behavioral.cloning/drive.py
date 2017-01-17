import base64
import argparse
import numpy as np
import socketio
# import eventlet
import eventlet.wsgi
# import time
from PIL import Image
# from PIL import ImageOps
from flask import Flask  # , render_template
from io import BytesIO

# from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array

# Fix error with Keras and TensorFlow
import tensorflow as tf
tf.python.control_flow_ops = tf

# my imports
from model_api import *
from image_handling import preprocess_image


sio = socketio.Server()
app = Flask(__name__)
model = None
prev_image_array = None


@sio.on('telemetry')
def telemetry(sid, data):
    # # The current steering angle of the car
    # driving_angle = float(data["steering_angle"])
    # # The current throttle of the car
    # throttle = float(data["throttle"])
    # # The current speed of the car
    speed = float(data["speed"])

    # The current image from the center camera of the car
    image_string = data["image"]
    image = Image.open(BytesIO(base64.b64decode(image_string))).convert('RGB')
    image_array = np.asarray(image)
    image_array = preprocess_image(image_array, IMAGE_SIZE_X, IMAGE_SIZE_Y, driving_mode=True)

    transformed_image_array = image_array[None, :, :, :]
    driving_angle = predict_angle(model, transformed_image_array)  # + lane_correction

    if speed < 20:
        throttle = 0.5
    else:
        throttle = 0.2

    print(driving_angle, throttle)
    send_control(driving_angle, throttle)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(driving_angle, throttle):
    sio.emit("steer", data={
        'steering_angle': driving_angle.__str__(),
        'throttle': throttle.__str__()
    }, skip_sid=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('--model', type=str,
                        help='Path to model definition json. Model weights should be on the same path.')
    args = parser.parse_args()
    if args.model:
        model = load_model(args.model)
    else:
        model = load_model()
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
