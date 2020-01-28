import numpy as np
import os
import json

"""
This program runs through all the yaml files in a dataset and adds the position data to a txt file so that it can be
plotted in excel.
"""

xyz = np.zeros((10000, 3))
image_index = 0
path = "/media/daniel/Daniel's HardDrive/Research/e3"

for r, d, f in os.walk(path):
    for file in f:
        if '.json' in file:
            print(file)
            with open(os.path.join(r,file)) as f2:
                json_data = json.load(f2)
            xyz[image_index,:] = json_data['position']

            if json_data['tracking_valid'] == False:
                print("Tracking INVALID for ",file)

            image_index += 1
xyz_trimmed = xyz[0:image_index,:]

np.savetxt("FullPositionTruth6.txt", xyz_trimmed)
