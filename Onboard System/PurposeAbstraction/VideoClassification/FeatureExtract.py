import numpy as np
import tensorflow as tf
import keras
from keras.models import Model
from tensorflow.python.keras.preprocessing import image
from keras.applications.inception_v3 import *
from keras.applications.mobilenet import *
import time
import cv2

import matplotlib.pyplot as plt

class FeatureExtractor:
    def __init__(self, weights=None):
       
        self.weights = weights
        if weights == None:
            model = InceptionV3(weights='imagenet', include_top=True)
        else:
            print("This feature not implemented yet!")
            exit()
        # (inputs=model.input, outputs=model.get_layer('avg_pool').output)
        # model.layers.pop()
        #model.layers.pop()
        self.CompleteModel = model
        self.FeatureExtractModel = Model(inputs=self.CompleteModel.input, outputs=self.CompleteModel.get_layer('avg_pool').output)
        self.imgDim = (299, 299)

    def ProcessImage(self, img):
        #x = cv2.resize(img, self.imgDim)
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        return x
       
    def GetFeatureMap(self, img):
        x = img
        # Get the prediction.
        features = self.FeatureExtractModel.predict(x)

        if self.weights is None:
            # For imagenet/default network:
            features = features[0]
        else:
            # For loaded network:
            features = features[0]

        return features
        
    def LoadImage(self, img):
        img = image.load_img(img, target_size=self.imgDim)
        return img

    def LoadProcessedVideo(self, vid, length = -1):
        success = 1;
        vid = cv2.VideoCapture(vid)
        vv = list()
        try:
            while length != 0:
                start_time = time.time() # start time of the loop
                success, img = vid.read()
                img = cv2.resize(img, (299, 299))
                if not success: break;
                img = self.ProcessImage(img)
                ff = self.GetFeatureMap(img)
                
                vv.append(ff)
                
                cv2.imshow('frame', ff)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                print("FPS: ", 1.0 / (time.time() - start_time)) # FPS = 1 / time to process loop
                length -= 1
        except (...):
            pass;
        return np.array(vv)

    def GetPredictions(self, img):
        y = self.CompleteModel.predict(img)
        print(decode_predictions(y))
        for index, res in enumerate(decode_predictions(y)[0]):
            print('{}. {}: {:.3f}%'.format(index + 1, res[1], 100 * res[2]))
