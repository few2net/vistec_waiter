#!/usr/bin/python env

import numpy as np

data = np.load('data.npy', allow_pickle=True)
data = data.item()
print(data)

