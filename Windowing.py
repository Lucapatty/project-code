import numpy as np

class Windower:
    def __init__(self, fs, window=200, overlap=0.5, feature="mean"):
        self.feature = feature
        self.windowlen = max(1, round((window * fs) / 1000))
        self.hoplen = max(1, round(self.windowlen * (1 - overlap)))
        self.buffer = np.zeros(self.windowlen)
        self.index = 0     
        self.filled = 0       
        self.count = 0        

    def window_processing(self, x):
        self.buffer[self.index] = x
        self.index = (self.index + 1) % self.windowlen
        self.filled = min(self.filled + 1, self.windowlen)
        self.count += 1

        if self.filled == self.windowlen and self.count >= self.hoplen:
            self.count = 0
            return self._compute_feature()
        return None

    def _compute_feature(self):
        if self.feature == "rms":
            return float(np.sqrt(np.mean(self.buffer ** 2)))
        return float(np.mean(self.buffer))