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
        [cos(p['2']), -1*sin(p['2']), 0],
        [sin(p['2']),    cos(p['2']), 0],
        [0,                0,         1]
    ])

    return R

def R23(p):

    R = Matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    return R

def P01():
    p = Matrix([0, 0, 0])
    return p

def P12(L1):
    p = Matrix([L1, 0, 0])
    return p

def P23(L2):
    p = Matrix([L2, 0, 0])
    return p

def I1(params):
    return Matrix([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

def I2(params):
    return Matrix([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

# def I1(params):
#     return Matrix([
#         [params['I1xx'], 0, 0],
#         [0, params['I1yy'], 0],
#         [0, 0, params['I1zz']]
#     ])

# def I2(params):
#     return Matrix([
#         [params['I2xx'], 0, 0],
#         [0, params['I2yy'], 0],
#         [0, 0, params['I2zz']]
#     ])

   