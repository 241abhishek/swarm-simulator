# def init(swarmsize, x, y, theta, a_ids):

#     import math
#     import random

#     x[0] = 0.0
#     y[0] = 0.0
#     theta[0] = 0.0
#     a_ids[0] = 0        


#     for i in range(1, swarmsize):
#         x[i] = random.uniform(-4.5, 4.5)
#         y[i] = random.uniform(-4.5, 4.5)
#         theta[i] = random.uniform(-4.5, 4.5)
#         a_ids[i] = i        

#     return x, y, theta, a_ids

def init(swarmsize,x,y,theta,a_ids):
    import math
    import random
    for i in range(swarmsize):
        x[i] = random.uniform(-4.5, 4.5)
        y[i] = random.uniform(-4.5, 4.5)
        theta[i] = random.uniform(-4.5, 4.5)
        a_ids[i] = i
    return x, y, theta, a_ids
