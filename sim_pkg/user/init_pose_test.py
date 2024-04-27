def init(swarmsize, x, y, theta, a_ids):
    import math
    import random

    for i in range(swarmsize):
        y[i] = random.uniform(-4.5,4.5)
        x[i] = random.uniform(-4.5,4.5)
        theta[i] = random.uniform(-math.pi,math.pi)
        a_ids[i] = i
    return x, y, theta, a_ids