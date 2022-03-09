# fixed-length FIFO class
# allows to continuously log information and iterate through it
# will be used to log odometry data

from collections import *

class FixedLengthFifo():
    def __init__(self, length):
        # initialize a deque as the data structure to represent
        # a fifo
        self.data = deque([], length)

    # push to the FIFO
    def push(self, item):
        self.data.appendleft(item)

    # return the length of the items in the fifo
    def length():
        return len(self.data)

    # returns the total amount of items that can be stored in the fifo
    def size():
        return self.data.maxlen

    # be able to index the fifo
    def __getitem__(self, index):
        return self.data[index]
