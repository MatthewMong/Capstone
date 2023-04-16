import statistics

import matplotlib.pyplot as plt
import numpy
import numpy as np
import pandas as pd
import scipy.signal
from scipy import signal
import math
from pyquaternion import Quaternion
from vqf import VQF
from numpy.fft import rfft,fft

# Import csv
from Capstone.app.IFFT import fourierFilt

df = pd.read_csv("C:/Users/jade-/OneDrive/Documents/UBC 2022-23/BMEG 457/Testing/March/sixth_20230324.csv")

# Retrieve raw data files
raw_acc_x = df[df.columns[0]]
raw_acc_y = df[df.columns[1]]
raw_gyr_z = df[df.columns[5]]
length = len(raw_gyr_z)
raw_acc_z = numpy.zeros(length)
raw_gyr_x = numpy.zeros(length)
raw_gyr_y = numpy.zeros(length)

time = df[df.columns[7]]
seconds = []

for i in range(0, length):
    seconds.append(time[i] / math.pow(10, 6) - time[0] / math.pow(10, 6))

seconds.append(seconds[-2] + seconds[-2] - seconds[-3])

raw_A = np.array([raw_acc_x, raw_acc_y, raw_acc_z])
raw_G = np.array([raw_gyr_x, raw_gyr_y, raw_gyr_z * math.pi / 180])

raw_A = np.ascontiguousarray(raw_A.transpose())
raw_G = np.ascontiguousarray(raw_G.transpose())

vqf = VQF(1 / 88)
out = vqf.updateBatch(raw_G, raw_A)
Q_init = out['quat6D']

Quat = []
distRad = []
distDeg = []
distRad.append(0)

for i in range(0, len(Q_init)):
    Quat.append(Quaternion(Q_init[i]))

for i in range(1, len(Q_init) - 1):
    distRad.append(2 * math.acos(np.real(Quat[i - 1] * Quat[i].conjugate)))

for i in range(0, len(distRad) - 1):
    distDeg.append(distRad[i] * 180 / math.pi)

plt.plot(distDeg)
plt.title('Quaternion angle over time')
plt.xlabel('time (s)')
plt.ylabel('angle (deg)')
plt.show()

b, a = signal.butter(4, 0.1, 'lowpass')

window_size = 100
numbers_series = pd.Series(distDeg)
windows = numbers_series.rolling(window_size)
moving_averages = windows.mean()
moving_averages = np.array(moving_averages.tolist())

thresh = 1.5
greater = np.where(moving_averages > thresh, 1, moving_averages)
greater[greater != 1] = 0
first = np.where(greater == 1)[0][0]
last = np.where(greater == 1)[0][-1]
print(first)
print(last)

plt.plot(distDeg)
plt.axvline(x=first, color='r')
plt.axvline(x=last, color='r')
plt.title('Quaternion angle over time')
plt.xlabel('time (s)')
plt.ylabel('angle (deg)')
plt.show()

# first = 3562
# last = 4516
length = last - first

fft_acc_x = fourierFilt(np.array(raw_acc_x[first:last]))
fft_acc_y = fourierFilt(np.array(raw_acc_y[first:last]))
fft_gyr_z = fourierFilt(np.array(raw_gyr_z[first:last]))

acc_x = signal.filtfilt(b, a, fft_acc_x)
acc_y = signal.filtfilt(b, a, fft_acc_y)
gyr_z = signal.filtfilt(b, a, fft_gyr_z)

acc_z = np.zeros(len(acc_x))
gyr_x = np.zeros(len(acc_x))
gyr_y = np.zeros(len(acc_x))

seconds = seconds[first+2:last]

A = np.array([acc_x, acc_y, acc_z])
G = np.array([gyr_x * math.pi / 180, gyr_y * math.pi / 180, gyr_z * math.pi / 180])

A = np.ascontiguousarray(A.transpose())
G = np.ascontiguousarray(G.transpose())

vqfF = VQF(1 / 88)
out = vqfF.updateBatch(G, A)
Q = out['quat6D']

Quat = []
distRad = []
distDeg = []

for i in range(0, len(Q)):
    Quat.append(Quaternion(Q[i]))

for i in range(1, len(Q)):
    distRad.append(2 * math.acos(np.real(Quat[i - 1] * Quat[i].conjugate)))

for i in range(0, len(distRad)):
    distDeg.append(distRad[i] * 180 / math.pi)

plt.plot(seconds, distDeg)
plt.title('Quaternion angle over time')
plt.xlabel('time (s)')
plt.ylabel('angle (deg)')
plt.show()

diffGyr = np.diff(signal.filtfilt(b, a, gyr_z)>0)
zeroCross = []

for i in range(0, len(diffGyr) - 1):
    if diffGyr[i]:
        zeroCross.append(i)

print(zeroCross)
print(len(zeroCross))
plt.plot(seconds, gyr_z[0:-1], '-', [seconds[i] for i in zeroCross], np.zeros(len(zeroCross)), 'o')
plt.title('Quaternion angle over time with zerocross')
plt.legend(['Angle', 'Zero'])
plt.xlabel('time (s)')
plt.ylabel('angle (deg)')
plt.show()
print("Number of kicks is: ", len(zeroCross)/2)
print("Kick Timing", [seconds[i] for i in zeroCross])

# Total Acceleration
totalAcc = np.sqrt(np.square(acc_x[0:-1]) + np.square(acc_y[0:-1]))
plt.plot(seconds, totalAcc, '-', [seconds[i] for i in zeroCross], np.zeros(len(zeroCross)), 'o')
plt.title('Total Acceleration')
plt.legend(['Acceleration', 'Zero'])
plt.xlabel('time (s)')
plt.ylabel('acceleration [m/s^2]')
plt.show()

# Quaternion amplitude
posAmpQ = []
negAmpQ = []
tPos = []
tNeg = []
for i in range(0, len(zeroCross)-2):
    if gyr_z[round((zeroCross[i+1]+zeroCross[i])/2)]>0:
        i1 = zeroCross[i]
        i2 = zeroCross[i+1]
        posAmpQ.append(sum(distDeg[i1:i2]))
        tPos.append(seconds[round((zeroCross[i+1]+zeroCross[i])/2)])
    else:
        i1 = zeroCross[i]
        i2 = zeroCross[i + 1]
        negAmpQ.append(sum(distDeg[i1:i2]))
        tNeg.append(seconds[round((zeroCross[i + 1] + zeroCross[i]) / 2)])

meanAmpQ = (np.array(posAmpQ)+np.array(negAmpQ))/2

plt.bar(tPos, posAmpQ, width=0.3)
plt.bar(tNeg, negAmpQ, width=0.3)
plt.xlabel("Time (s)")
plt.ylabel('Amplitude (deg)')
plt.title("Amplitude per Kick: Up vs. Down")
plt.legend("positive","negative")
plt.show()

tAvg = (np.array(tPos)+np.array(tNeg))/2
plt.bar(tAvg, meanAmpQ)
plt.xlabel("Time (s)")
plt.ylabel('Amplitude (deg)')
plt.title("Amplitude per Kick")
plt.show()

print("Average Kick Amplitude is: ", statistics.mean(meanAmpQ))

#Jerk Cost
posAmpJ = []
negAmpJ = []

for i in range(0, len(zeroCross)-2):
    if gyr_z[round((zeroCross[i+1]+zeroCross[i])/2)]>0:
        i1 = zeroCross[i]
        i2 = zeroCross[i+1]
        posAmpJ.append(math.pow(max(totalAcc[i1:i2]), 2)/(seconds[i2]-seconds[i1]))
    else:
        i1 = zeroCross[i]
        i2 = zeroCross[i + 1]
        negAmpJ.append(math.pow(max(totalAcc[i1:i2]), 2) / (seconds[i2] - seconds[i1]))

plt.bar(tPos, posAmpJ, width=0.3)
plt.bar(tNeg, negAmpJ, width=0.3)
plt.xlabel("Time (s)")
plt.ylabel('Jerk Cost (m^2/s^5)')
plt.title("Jerk Cost per Kick: Up vs. Down")
plt.legend("positive", "negative")
plt.show()

#Velocity
posAmpV = []
negAmpV = []

for i in range(0, len(zeroCross)-2):
    if gyr_z[round((zeroCross[i+1]+zeroCross[i])/2)]>0:
        i1 = zeroCross[i]
        i2 = zeroCross[i+1]
        posAmpV.append(abs(np.trapz(seconds[i1:i2], totalAcc[i1:i2])))
    else:
        i1 = zeroCross[i]
        i2 = zeroCross[i + 1]
        negAmpV.append(abs(np.trapz(seconds[i1:i2], totalAcc[i1:i2])))

plt.bar(tPos, posAmpV, width=0.3)
plt.bar(tNeg, negAmpV, width=0.3)
plt.xlabel("Time (s)")
plt.ylabel('Velocity (m/s)')
plt.title("Absolute Velocity per Kick: Up vs. Down")
plt.legend("positive", "negative")
plt.show()