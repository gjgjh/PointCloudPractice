import numpy as np
import pycluster

if __name__ == '__main__':
    x = np.array([[1, 2], [1.5, 1.8], [5, 8], [8, 8], [1, 0.6], [9, 11], [100, 12], [90, 11], [99, 13]])
    spectralCluster = pycluster.SpectralCluster(3, 4, 0.001, 300)
    spectralCluster.fit(x)

    cat = spectralCluster.predict(x)
    print(cat)
