#!/usr/bin/env python
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
pwd = os.getcwd()

data = pd.read_csv(pwd+"/ref_v.csv")
rx = data["n"]
ry = data["v"]

data2 = pd.read_csv(pwd+"/v.csv")
x_new = data2["n"]
y_new = data2["v"]

plt.plot(rx, ry, x_new, y_new)

plt.show()

