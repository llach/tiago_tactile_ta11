import os
import pickle

import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

sns.set_theme()

with open("{}/resp_times.pkl".format(os.environ['HOME']), "rb") as f:
    rt = pickle.load(f)

sns.histplot(rt)
plt.xlabel("Time [microseconds]")
plt.title("Strain Gauge Readout Delay")
print(f"mean {np.mean(rt)} std {np.std(rt)} median {np.median(rt)}")

plt.show()
