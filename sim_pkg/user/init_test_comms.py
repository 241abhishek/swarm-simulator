def init(swarmsize, x, y, theta, a_ids):

    import math

    x[0] = 2.0
    y[0] = 0.0
    theta[0] = math.pi
    a_ids[0] = 0
    x[1] = -2.0
    y[1] = 0.0
    theta[1] = 0.0
    a_ids[0] = 1

    return x, y, theta, a_ids


