def init(swarmsize,x,y,theta,a_ids):
    import math
    import random

    arena_length = 15.0
    arena_width = 15.0

    # intialize the position of the sheep in the upper right quadrant
    q = random.randint(1, 3)
    if q == 1:
        x[0] = random.uniform(-(arena_width/2.0), 0.0)
        y[0] = random.uniform(0.0, (arena_length/2.0))
    elif q == 2:
        x[0] = random.uniform(-(arena_width/2.0), 0.0)
        y[0] = random.uniform(-(arena_length/2.0), 0.0)
    elif q == 3:
        x[0] = random.uniform(0.0, (arena_width/2.0))
        y[0] = random.uniform(-(arena_length/2.0), 0.0)
    theta[0] = random.uniform(-math.pi, math.pi)
    a_ids[0] = 1 # id for shepherd

    # initialze the position of the shepherd in one of the other three quadrants
    for i in range(1, swarmsize):
        x[i] = random.uniform(0.0, (arena_width/2.0))
        y[i] = random.uniform(0.0, (arena_length/2.0))
        theta[i] = random.uniform(-math.pi, math.pi)
        a_ids[i] = 0 # id for sheep

    return x, y, theta, a_ids 