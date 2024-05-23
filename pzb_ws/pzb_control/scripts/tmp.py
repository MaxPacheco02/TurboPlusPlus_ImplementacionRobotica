#!/usr/bin/env python3
import numpy as np
import math

def get_R(ang):
    return np.array([
        [math.cos(ang), -math.sin(ang), 0],
        [math.sin(ang), math.cos(ang), 0],
        [0, 0, 1]])

print(get_R(math.pi).dot(np.array([1,0,0])))