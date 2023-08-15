import sympy
import numpy as np
from sympy import Matrix
from sympy import sin, cos


def R01(p):

    R = Matrix([
        [cos(p['1']),  -1 * sin(p['1']), 0],
        [sin(p['1']),       cos(p['1']), 0],
        [0,                0,            1]
    ])

    return R
    
def R12(p):

    R = Matrix([
        [cos(p['2']), -1 * sin(p['2']),   0],
        [0,                0,             1],
        [-1 * sin(p['2']),      -1 * cos(p['2']),   0]
    ])

    return R

def R23(p):
    
    R = Matrix([
        [cos(p['3']), -1*sin(p['3']), 0],
        [sin(p['3']),    cos(p['3']), 0],
        [0,                0,         1]
    ])

    return R

def R34(p):

    R = Matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    return R

def P01(L1):
    p = Matrix([0, 0, L1])
    return p

def P12():
    p = Matrix([0, 0, 0])
    return p

def P23(L2):
    p = Matrix([0, -1 * L2, 0])
    return p

def P34(L3):
    p = Matrix([0, -L3, 0])
    return p

def I1(params):
    return Matrix([
        [params['I1xx'], 0, 0],
        [0, params['I1yy'], 0],
        [0, 0, params['I1zz']]
    ])

def I2(params):
    return Matrix([
        [params['I2xx'], 0, 0],
        [0, params['I2yy'], 0],
        [0, 0, params['I2zz']]
    ])

def I3(params):
    return Matrix([
        [params['I3xx'], 0, 0],
        [0, params['I3yy'], 0],
        [0, 0, params['I3zz']]
    ])



   