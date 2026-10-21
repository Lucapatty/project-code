import numpy as np

class Butterworth:
    def __init__(self, fs=2, fc=2, order=4):
        self.fs = fs
        self.fc = fc
        self.order = order 
        Wc = (2*fs*np.tan(np.pi*(fc/fs)))

        sos = []

        for i in range (order // 2):
            tetha = (2*i+1) * np.pi / (2*order)
            p = -np.sin(tetha) + 1j * np.cos(tetha)
            p_hp = Wc / p
            zp = ((2*fs+p_hp)/(2*fs-p_hp))
            a1 = (-2* zp.real)
            a2 = (abs(zp)**2)
            g = ((1 - a1 + a2)/4)                     
            sos.append([g, -2 * g, g, 1.0, a1, a2])

        if order % 2 == 1:
            zp = ((2*fs-Wc)/(2*fs+Wc))
            a1 = -zp
            g = ((1 - a1) / 2)
            sos.append([g, -g, 0.0, 1.0, a1, 0.0])

        self.sos = np.array(sos)
        self.z = np.zeros((self.sos.shape[0], 2))

    def butterworth_processing(self, x):
        for i, (b0, b1, b2, _, a1, a2) in enumerate (self.sos):
            y = b0*x + self.z[i, 0]
            self.z[i, 0] = (b1*x) - (a1*y) + self.z[i, 1]
            self.z[i, 1] = (b2*x) - (a2*y)
            x = y
        return x





class Rectify:
    def rectification_processing(self, x):
        return abs(x)





class MovingAverageFilter:
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




class Normalizer:
    def __init__(self, percentage=95):
        self.percentage = percentage
        self.rest = 0.0        
        self.max = None       
        self.eps = 1e-6

    def calibrate_rest(self, rest):
        self.rest = float(np.mean(rest))

    def calibrate_max(self, active):
        self.max = float(np.percentile(active, self.percentage))

    def normalizer_processing(self, x):
        normalize = (x - self.rest) / (self.max - self.rest + self.eps)
        return min(max(normalize, 0.0), 1.0) 





class Windower:
    def __init__(self, fs, window=200, overlap=0.5, feature="mean"):
        self.feature = feature
        self.windowlen = max(1, round(window * fs / 1000))
        self.hoplen = max(1, round(self.windowlen * (1 - overlap)))
        self.buffer = np.zeros(self.windowlen)
        self.index = 0       
        self.filled = 0      
        self.count = 0       


    def __init__(self, fs, window=200, overlap=0.5, feature="mean"):
        self.feature = feature
        self.windowlen = max(1, round((window * fs) / 1000))
        self.hoplen = max(1, round(self.windowlen * (1 - overlap)))
        self.buffer = np.zeros(self.windowlen)
        self.index = 0       
        self.filled = 0       
        self.count = 0 

    def update(self, x):
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