#!/usr/bin/env python

# Written by Mikael Glamheden
# 2019-06-04

class Data_log():
    """Object to log data about vehicle state in."""
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []

    def append_data(self,x,y,yaw,v):
        self.x.append(x)
        self.y.append(y)
        self.yaw.append(yaw)
        self.v.append(v)

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_yaw(self):
        return self.yaw

    def get_v(self):
        return self.v
