import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 10, 500)
B = 10
C = 5
scale = 1 + np.sin(B * t + np.pi/2) * np.exp(-C * t)
scale[t >= 5.0] = 1.0
ref = 0.38919
w = ref * scale
plt.plot(t, w)
plt.show()