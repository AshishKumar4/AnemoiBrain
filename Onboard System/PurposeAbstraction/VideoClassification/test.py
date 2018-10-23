import numpy as np 
import tensorflow as tf 
import keras 
from tensorflow.python.keras.preprocessing import image
from keras.applications.inception_v3 import *

from Classifier import *
from FeatureExtract import *

import cv2
import pickle

DATA_URL = 'http://download.tensorflow.org/models/image/imagenet/inception-2015-12-05.tgz'

mm = FeatureExtractor()
model = mm.CompleteModel
gardien = Classifier()

#vidcap = cv2.VideoCapture('bigbunny.mp4')
success = 1;# False;#True
while success:
    #success,image = vidcap.read()
    image = mm.LoadImage("image.jpg")
    #ff = mm.GetFeatureMap(inn)
    inn = mm.ProcessImage(image)
    
    print(inn, inn.shape)
    ff = mm.GetFeatureMap(inn)
    print(ff, ff.shape)
    success = False
vv = mm.LoadProcessedVideo('bigbunny.mp4')
print(vv.shape)