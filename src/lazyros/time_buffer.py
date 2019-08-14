import numpy as np
import bisect


class TimeBuffer(object):
    def __init__(self, capacity=100):
        self._buffer = []
        self._capacity = capacity

    def append(self, time, message):
        print(self._buffer)
        bisect.insort_right(self._buffer, (time, message))
        if len(self._buffer) > self._capacity:
            self._buffer = self._buffer[-self._capacity:]

    def get_latest(self, time):
        idx = bisect.bisect(self._buffer, (time, None))
        if self._buffer[idx][0] > time:
            idx = idx - 1
        return self._buffer[idx][1]

    def get_closest(self, time):
        idx = bisect.bisect(self._buffer, (time, None))
        right_item = self._buffer[idx]
        left_item = self._buffer[max(idx-1,0)]
        if abs(time - right_item[0]) < abs(time - left_item[0]):
            return right_item[1]
        else:
            return left_item[1]

    def __len__(self):
        return len(self._buffer)
