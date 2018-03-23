import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

class Plotter(object):
    def __init__(self):
        self.truedists = [] #fifo of true distances
        self.advdists = [] #fifo of adversarial distances
        self.xvals = []
        self.maxlen = 50
        self.currlen = 0
        #plt.figure()
        self.plt = plt
        self.plt.ion()    
    
    def setValues(self, true_dist, adv_dist):
        if self.currlen >= self.maxlen:
            self.truedists.pop(0)
            self.advdists.pop(0)
            self.plt.clf()

        if self.currlen < self.maxlen: 
            self.currlen = self.currlen + 1
            self.xvals.append(self.currlen)

        self.truedists.append(true_dist)
        self.advdists.append(adv_dist)
        self.plt.plot(self.xvals, self.truedists, color='b')
        self.plt.plot(self.xvals, self.advdists, color='r')
        self.plt.ylabel('distances (cm)')
        self.plt.xlabel('frames')
        self.plt.pause(0.01)


     

        
