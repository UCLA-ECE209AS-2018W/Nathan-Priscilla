'''Warp Functions'''

import numpy as np
from skimage import transform

def fisheye(xy):
    center = np.mean(xy, axis=0)
    center = [400.0,200.0]
    xc, yc = (xy - center).T

    # Polar coordinates
    r = np.sqrt(xc**2 + yc**2)
    theta = np.arctan2(yc, xc)

    #r = 0.8 * np.exp(r**(1/2.1) / 1.8)
    #r = 4 * np.exp(r**(1/2.1) / 1.8) #reduces the effect 
    #r = 2.1 * np.exp(r**(1/2.1) / 3.5) #level 1 
    r = 2.1 * np.exp(r**(1/1.5)/11.3) #level 2
    #r = 2.1 * np.exp(r**(1/3.0)/1.45) #level 3
    #r = 2.1 * np.exp(r**(1/3.0)/1.2) #level 3

    return np.column_stack((
        r * np.cos(theta), r * np.sin(theta)
        )) + center


'''Define your warp function below'''

