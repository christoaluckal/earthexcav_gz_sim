import numpy as np
from scipy.interpolate import CubicSpline


def bucket_cmd(x):
        if abs(x) <= 0.45: return 0.0
        if x < -0.9: return -0.7464773197463939
        elif x < -0.8: return -0.7556077862660108
        elif x < -0.7: return -0.7475886335802124
        elif x < -0.6: return -0.6943159340316355
        elif x < -0.5: return -0.3478137666597351
        elif x < -0.45: return -0.11074819014084174
        elif x < 0.45: return -0.03170411937007261
        elif x < 0.5: return 0.07074540207098638
        elif x < 0.6: return 0.1531096691607231
        elif x < 0.7: return 0.494843938147395
        elif x < 0.8: return 0.9654822056154688
        elif x < 0.9: return 1.1323096288265486
        elif x < 1.0: return 1.124019310302267
        else: return 1.1221103605940879

signal = np.arange(-1.0, 1.0, 0.1)
power =  np.array([bucket_cmd(x) for x in signal])
cumm_power_diff = np.cumsum(np.abs(np.diff(power)))

print(cumm_power_diff)
