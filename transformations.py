# Funktionen zur Berechnung von Koordinatentransformationen in 2D und 3D
#
# M. Blaich; 13.10.2023;

import numpy as np


def rot(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    

def rotx(theta):
    return np.matrix([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta),  np.cos(theta)]
        ])
    


def roty(theta):
    return np.matrix([
        [np.cos(theta),  0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])


def rotz(theta):
    return np.matrix([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])


def rot2trans(r):
    dim = r.shape[0]
    t = np.identity(dim + 1)
    t[:dim, :dim] = r
    return t
    


def trans(t):
    dim = t.size
    m = np.identity(dim + 1)
    t.transpose()
    m[:dim, dim] = t
    return m
