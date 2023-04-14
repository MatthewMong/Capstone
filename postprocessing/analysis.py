import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tsmoothie.smoother import LowessSmoother, ExponentialSmoother, SpectralSmoother
from scipy import signal
from scipy.interpolate import interp1d


dataLabels = ['accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ', 'barom', 'ns']
data = pd.read_csv('C:/Users/ascar/OneDrive/Desktop/Capstone/data7.txt', names=dataLabels)

subsample = data[data['ns'].between(130000000, 170000000)]
# subsample = data
subsample = subsample[subsample['gyroZ'].abs() > 100]

subsample = subsample[subsample['gyroZ'].abs() != 0]

smoother = LowessSmoother(smooth_fraction=0.01, iterations=1)
smoother.smooth(subsample['gyroZ'])
low, up = smoother.get_intervals('prediction_interval')
zero_crossings = np.where(np.diff(np.sign(smoother.smooth_data[0])))[0]

zero_values = subsample.iloc[zero_crossings]['gyroZ']
# number of crosses
# differentsigns = np.sign(zero_values).diff().ne(0)
# differentsigns = differentsigns[differentsigns == True]
print(subsample.iloc[zero_crossings][['ns','gyroZ']])
print(len(zero_values))
plt.plot(smoother.smooth_data[0], linewidth=3, color='blue')
plt.fill_between(range(len(smoother.data[0])), low[0], up[0], alpha=0.3)
# plt.plot(subsample['gyroZ'])
plt.show()
