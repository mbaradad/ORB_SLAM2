import orb_slam2
import numpy as np

img = np.zeros((300,300,3), dtype='uint8')
orb_slam2 = orb_slam2.OrbSlamSystem()

orb_slam2.TrackMonocular(img, 0.0)