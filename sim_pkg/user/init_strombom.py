def init(swarmsize,x,y,theta,a_ids):
    import math
    import random
    for i in range(swarmsize):
        x[i] = random.uniform(-4.5, 4.5)
        y[i] = random.uniform(-4.5, 4.5)
        theta[i] = random.uniform(-4.5, 4.5)
        a_ids[i] = i
    return x, y, theta, a_ids