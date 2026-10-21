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

    def processing(self, x):
        for i, (b0, b1, b2, _, a1, a2) in enumerate (self.sos):
            y = b0*x + self.z[i, 0]
            self.z[i, 0] = (b1*x) - (a1*y) + self.z[i, 1]
            self.z[i, 1] = (b2*x) - (a2*y)
            x = y
        return x