import numpy as np
import pycluster

if __name__ == '__main__':
    x = np.array([[1, 2], [1.5, 1.8], [5, 8], [8, 8], [1, 0.6], [9, 11]])
    kmeans = pycluster.Kmeans(2, 0.001, 300)
    kmeans.fit(x)

    cat = kmeans.predict(x)
    print(cat)
