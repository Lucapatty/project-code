import numpy as np
class MovingAveregeFilter:
    def __init__(self, fs, window=250):
        self.windowlen = max(1, round((window*fs)/1000))
        self.buffer = np.zeros(self.windowlen)
        self.index = 0
        self.sum = 0.0

    def movingaverage_processing(self, x):
        self.sum += x - self.buffer[self.index]
        self.buffer[self.index] = x
        self.index = (self.index + 1) % self.windowlen
        output = self.sum / self.windowlen
        return output
