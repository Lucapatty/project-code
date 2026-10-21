import numpy as np

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