import numpy as np
import matplotlib.pyplot as plt

filename1 = "descriptor1.txt"
filename2 = "descriptor2.txt"
descriptor1 = np.loadtxt(filename1)
descriptor2 = np.loadtxt(filename2)

plt.title("descriptor1")
plt.ylim([0, 1])
plt.plot(descriptor1)
plt.show()

plt.close()
plt.title("descriptor2")
plt.ylim([0, 1])
plt.plot(descriptor2)
plt.show()
