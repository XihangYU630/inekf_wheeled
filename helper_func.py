import numpy as np

def skew(x):
    mat = np.array([[0,-x(3),x(2)],
        [x(3),0,-x(1)],
        [-x(2),x(1),0]])
    return mat

def wedge(x):
    # wedge operation for se2(3)
    xhat = np.array([[0, x[2], x[1], x[3], x[6]],
                    [x[2], 0, -x[0], x[4], x[7]],
                    [-x[1], x[0], 0, x[5], x[8]],
                    [0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0]])
    return xhat



