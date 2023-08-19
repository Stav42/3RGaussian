import numpy as np
import sympy
from sympy import symbols, hessian
from sympy.vector import gradient
from sympy import cos, sin
# from Transformation import R01, R12, R23, R34, P01, P12, P23, P34, I1, I2, I3
from sympy import Matrix, Function, diff
from sympy import simplify, latex
from sympy.physics.mechanics import mprint, mlatex

t = symbols('t')
theta1 = Function('theta1')(t); theta2 = Function('theta2')(t); theta3 = Function('theta3')(t)
theta_d1 = diff(theta1, t); theta_d2 = diff(theta2, t); theta_d3 = diff(theta3, t)
theta_dd1 = diff(theta_d1, t); theta_dd2 = diff(theta_d2, t); theta_dd3 = diff(theta_d3, t)
L1, L2, L3 = symbols('L1 L2 L3')
m1, m2, m3, I1_bar, I1, I2_bar, I2, I3_bar, I3 = symbols('m1 m2 m3 I1_bar I1 I2_bar I2 I3_bar I3')
g = symbols('g')

def gradient(f, params):
    diff1 = diff(f, params[0])
    diff2 = diff(f, params[1])
    diff3 = diff(f, params[2])
    
    return Matrix([diff1, diff2, diff3])

R01 = Matrix([[cos(theta1), -sin(theta1), 0], [sin(theta1), cos(theta1), 0], [0, 0, 1]])
P01 = Matrix([0, 0, L1])
T01 = Matrix([[cos(theta1), -sin(theta1), 0, 0], [sin(theta1), cos(theta1), 0, 0], [0, 0, 1, L1], [0, 0, 0, 1]])

R12 = Matrix([[cos(theta2), -sin(theta2), 0], [0, 0, 1], [-sin(theta2), -cos(theta2), 0]])
P12 = Matrix([0, 0, 0])
T12 = Matrix([[cos(theta2), -sin(theta2), 0, 0], [0, 0, 1, 0], [-sin(theta2), -cos(theta2), 0, 0], [0, 0, 0, 1]])

R23 = Matrix([[cos(theta3), -sin(theta3), 0], [sin(theta3), cos(theta3), 0], [0, 0, 1]])
P23 = Matrix([L2, 0, 0])
T23 = Matrix([[cos(theta3), -sin(theta3), 0, L2], [sin(theta3), cos(theta3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

v00 = Matrix([0, 0, 0])
w00 = Matrix([0, 0, 0])

w11 = R01.T * w00 + Matrix([0, 0, theta_d1])
v11 = R01.T * (v00 + w00.cross(P01))
vc1 = v11 + w11.cross(Matrix([0, 0, -L1/2]))
PC10_der = T01 * (Matrix([0, 0, -L1/2, 1]))
PC10 = Matrix([0, 0, L1/2, 1])
# print("Position of COM1 wrt global frame: ", PC10_der)
I11 = Matrix([[I1, 0, 0], [0, I1, 0], [0, 0, I1_bar]])

w22 = R12.T * w11 + Matrix([0, 0, theta_d2])
v22 = R12.T * (v11 + w11.cross(P12))
vc2 = v22 + w22.cross(Matrix([L2/2, 0, 0]))
PC20_der = T01 * T12 * Matrix([L2/2, 0, 0, 1])
I22 = Matrix([[I2_bar, 0, 0], [0, I2, 0], [0, 0, I2]])
# print("\nPosition of COM2 wrt global frame: ", simplify(PC20_der))

w33 = R23.T * w22 + Matrix([0, 0, theta_d3])
v33 = R23.T * (v22 + w22.cross(P23))
vc3 = v33 + w33.cross(Matrix([L3/2, 0, 0]))
PC30_der = T01 * T12 * T23 * Matrix([L3/2, 0, 0, 1])
I33 = Matrix([[I3_bar, 0, 0], [0, I3, 0], [0, 0, I3]])
# print("\nPosition of COM3 wrt global frame: ", simplify(PC30_der))
G = Matrix([0, 0, -g, 0])

print("Velocity of COM1 in frame 1: ", vc1)
print("\nVelocity of COM2 in frame 2: ", vc2)
print("\nVelocity of COM3 in frame 3: ", simplify(vc3))

KE_code = 0.5*m1*vc1.T*vc1 + 0.5*m2*vc2.T*vc2 + 0.5*m3*vc3.T*vc3 + 0.5 * w11.T * (I11 * w11) + 0.5 * w22.T * (I22 * w22) + 0.5 * w33.T * (I33 * w33)
PE_code = -m1 * G.T * PC10 - m2 * G.T * PC20_der - m3 * G.T * PC30_der  


KE_1 = 0.5 * theta_d1 ** 2 * I1_bar + 0.5 * ((I2_bar * (sin(theta2)*theta_d1)**2) + I2*(cos(theta2)*theta_d1)**2 + I2*(theta_d2)**2) + 0.5 * (I3_bar*(sin(theta2+theta3)*theta_d3)**2 + I3 * (cos(theta2 + theta3)*theta_d1)**2 + I3*(theta_d2+theta_d3)**2)
KE_2 = 0.5 * m2 * ((cos(theta2) * theta_d1 * L1/2)**2 + (L2/2 * theta_d2)**2) + 0.5 * m3 * ((L2*theta_d2*sin(theta3))**2 + (L2*theta_d2*cos(theta3)+L3/2*(theta_d2+theta_d3))**2 + (-theta_d1*cos(theta2)-cos(theta2 + theta3)*theta_d1*cos(theta3)/2)**2)
KE = KE_1 + KE_2

PE = m1 * g * L1/2 + m2 * g* (sin(theta2)*L2/2+L1) + m3 * g * (sin(theta2)*cos(theta3)*L3/2 + sin(theta2)*L2 + cos(theta2)*sin(theta3)*L2+L1)


M = hessian(KE_code[0, 0], [theta_d1, theta_d2, theta_d3])
M = simplify(M)
print("\n Mass Matrix: ", simplify(M))
G = gradient(PE_code[0, 0], [theta1, theta2, theta3])
G = simplify(G)
print("\n Gravity vector is: ", simplify(G))
H = Matrix(gradient(KE_code[0, 0], [theta_d1, theta_d2, theta_d3])).jacobian(Matrix([theta1, theta2, theta3])) * Matrix([theta_d1, theta_d2, theta_d3]) - Matrix(gradient(KE_code[0, 0], [theta1, theta2, theta3]))
H = simplify(H)
print("\n NonLinear vector is: ", simplify(H))

f = open('3RDynamics.txt', 'w')
f.write(mlatex(M)+'\n')
f.write(mlatex(G)+'\n')
f.write(mlatex(H)+'\n')
