import csv
import numpy as np
from numpy import genfromtxt
import os
import json

moCapSync = 6.53   #373.36
imageSync = 331.91 #2163.36

imageStart = imageSync - moCapSync


# Load in cleaned csv file into numpy array
moCapData = genfromtxt('Take3cleaned.csv', delimiter=',') #time in seconds

# Clean up moCapData
i = 0
while i < np.size(moCapData,0):
    if np.isnan(moCapData[i,1]):
        moCapData = np.delete(moCapData,i,0)
        i -= 1
    i += 1

moCapIndex = 0
# Load in json timestamp and correct by image time-offset
for r, d, f in os.walk("./"):
    for file in f:
        if '.json' in file:
            with open(file, "r") as jsonFile:
                data = json.load(jsonFile)

            print("imageTimeStamp:",data["timestamp"])
            data["timestamp"] = data["timestamp"] - imageStart
            print("imageTimeStamp:",data["timestamp"])
            imageTimeStamp = data["timestamp"]
            moCapTimeStamp = moCapData[moCapIndex,0]

            print("Looking for sync (",imageTimeStamp,")")
            # Find closest time sync
            while moCapTimeStamp < imageTimeStamp:
                moCapIndex += 1
                moCapTimeStamp = moCapData[moCapIndex,0]

            print("Found higher MoCapTimeStamp:",moCapData[moCapIndex,0])
            print("Difference to Lower:",imageTimeStamp - moCapData[moCapIndex-1,0])
            print("Difference to Higher:",moCapData[moCapIndex,0] - imageTimeStamp)
            # If the lower moCapTimeStamp is closer to the imageTimeStamp
            if (moCapData[moCapIndex,0] - imageTimeStamp) > (imageTimeStamp - moCapData[moCapIndex-1,0]):
                print("Choosing lower time")
                moCapIndex -= 1

            data["syncError"] = abs(moCapData[moCapIndex,0] - imageTimeStamp)
            data["position"] = [moCapData[moCapIndex,5], moCapData[moCapIndex,6], moCapData[moCapIndex,7]]
            data["rotation"] = [moCapData[moCapIndex,1],moCapData[moCapIndex,2],moCapData[moCapIndex,3],moCapData[moCapIndex,4]]






            with open(file, "w") as jsonFile:
                json.dump(data, jsonFile, indent=4)

            moCapIndex = 0



# Walk through numpy array until closest timestamp is found

# add pose data to json file
