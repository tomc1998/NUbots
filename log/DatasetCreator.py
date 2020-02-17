import csv
from numpy import genfromtxt

# Load in cleaned csv file into numpy array
moCapData = genfromtxt('Take4cleaned.csv', delimiter=',')

print(moCapData[0:4,0])
moCapData[:,0] = int(moCapData[0,:]*1e9)
print(moCapData[0:4,0])

# Correct time by MoCap time-offset

# Load in json timestamp and correct by image time-offset

# Walk through numpy array until closest timestamp is found

# add pose data to json file
