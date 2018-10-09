from keras.models import Sequential
from keras.layers import Dense, Activation
from keras.layers import LSTM

class Classifier:
    def __init__(self):
        print("Yo")
    
    def CreateModel(self):
        frames_num=15
        count = 0
        self.chunk_size = 2048
        self.n_chunks = frames_num
        self.rnn_size = 512
        self.model = Sequential()
        self.model.add(LSTM(self.rnn_size, input_shape=(self.n_chunks, self.chunk_size)))
        self.model.add(Dense(1024))
        self.model.add(Activation('relu'))
        self.model.add(Dense(50))
        self.model.add(Activation('sigmoid'))
        self.model.add(Dense(3))
        self.model.add(Activation('softmax'))
        self.model.compile(loss='mean_squared_error', optimizer='adam',metrics=['accuracy'])
        self.data =[]
        self.target=[]
        self.epoch = 1500
        self.batchS = 100
        self.model.summary()
        return self.model
        
    def TrainModel(self):
        self.model.fit(self.data, self.target, epochs=self.epoch, batch_size=self.batchS, verbose=1)

    def Save(self):
        self.model.save("rnn.h5", overwrite=True)