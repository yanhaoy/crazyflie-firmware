import pickle
import matplotlib.pyplot as plt
import numpy as np
import sys

assert len(sys.argv) == 2
file_name = str(sys.argv[1])

a = 2.130295 * 1e-11
b = 1.032633 * 1e-6
c = 5.484560 * 1e-4

with open(file_name, 'rb') as file:
    log_dict = pickle.load(file)

for i, key in enumerate(log_dict):
    print(key)
    data = np.array(log_dict[key])

    # if i == 2:
    #     data -= 0.3

    if (i >= 3 and i <= 5) or (i >= 9 and i <= 11):
        data /= 180
        data *= np.pi

    if i >= 12:
        data = a*data**2 + b*data + c - 27*1e-3*9.81/4

    plt.figure()
    plt.plot(data)

plt.show()
