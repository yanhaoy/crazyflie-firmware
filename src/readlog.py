import pickle
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
import sys

assert len(sys.argv) == 2
file_name = str(sys.argv[1])

x = np.empty([12, 500])
u = np.empty([4, 500])

a = 2.130295 * 1e-11
b = 1.032633 * 1e-6
c = 5.484560 * 1e-4

with open(file_name, 'rb') as file:
    log_dict = pickle.load(file)

for i, key in enumerate(log_dict):
    print(key)
    data = np.array(log_dict[key])

    if i == 2:
        data -= 0.3
    elif (i >= 3 and i <= 5) or (i >= 9 and i <= 11):
        data /= 180
        data *= np.pi
    elif i >= 12:
        data = a*data**2 + b*data + c - 28*1e-3*9.81/4

    plt.figure()
    plt.plot(data)

    data = scipy.signal.medfilt(data[750:1250], kernel_size=9)
    print("mean: ", np.mean(data), "std: ", np.std(data), "max: ", np.max(np.abs(data)))

    if i<12:
        x[i, :] = data
    else:
        u[i-12, :] = data

K = -u@np.linalg.pinv(x)

print(K)
print(x[:, 0])
print(u[:, 0])

plt.show()
