import scipy
from numpy.fft import rfft,fft
from scipy.fft import irfft, ifft
import numpy as np
import matplotlib.pyplot as plt

def fourierFilt(raw_sig):
    freq = 88
    l = len(raw_sig)
    t = 1/freq
    y = rfft(raw_sig)
    p2 = abs(y/l)
    p1 = p2[0:int(l/2)]
    p1[1:-2] = 2*p1[1:-2]
    pX = np.sort(p1)
    lim = pX[-1-int(l/2*0.05)]
    y[p2 < lim] = 0
    val = irfft(y)
    return val