import numpy as np
import sympy
from sympy import symbols
from sympy import cos, sin
from Transformation import R01, R12, R23, R34, P01, P12, P23, P34, I1, I2, I3
from sympy import Matrix, Function, diff
from sympy import simplify, latex
from sympy.physics.mechanics import mprint, mlatex
from lagrangian_helper import obtain_links, Inertia, mass, theta, theta_dot, theta_ddot, functions

L1, L2, L3 = symbols('L1 L2 L3')
m1, m2, m3, I1xx, I1yy, I1zz, I2xx, I2yy, I2zz,  I3xx, I3yy, I3zz = symbols('m1 m2 m3 I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz')
g = symbols('g')

t = symbols('t')
theta1 = Function('theta1')(t); theta2 = Function('theta2')(t); theta3 = Function('theta3')(t)
theta_d1 = diff(theta1, t); theta_d2 = diff(theta2, t); theta_d3 = diff(theta3, t)
theta_dd1 = diff(theta_d1, t); theta_dd2 = diff(theta_d2, t); theta_dd3 = diff(theta_d3, t)


links = obtain_links()

def T(R, P):
    matr = Matrix([
        [R[0, 0], R[0, 1], R[0, 2], P[0]],
        [R[1, 0], R[1, 1], R[1, 2], P[1]],
        [R[2, 0], R[2, 1], R[2, 2], P[2]],
        [0,       0,       0,          1]
    ])

    return matr

def kinetic_energy(link):

    ke = 0.5 * mass['mass_'+str(link['link'])] * link['v_c'].T * link['v_c'] + 0.5 * link['w'].T * mass['I'+ str(link['link'])] * link['w']
    # print(ke)
    return ke

def sum_of_ke(link1, link2, link3):
    return kinetic_energy(link1) + kinetic_energy(link2) + kinetic_energy(link3)

def potential_energy(link, T):
    name = link['link'] - 1
    PC = functions['PC'+str(name)]

    PC = Matrix([PC[0], PC[1], PC[2], 1])
    R = functions['R'+str(name)+str(name+1)].T
    P = functions['P'+str(name)+str(name+1)]

    P0 = T * PC
    grav = Matrix([0, 0, -g, 0])

    u = -mass['mass_' + str(name + 1)] * grav.T * P0
    # print(u)

    return u

def sum_of_pe(link1, link2, link3):

    T01 = T(R01(theta), P01(L1))
    T12 = T(R12(theta), P12())
    T23 = T(R23(theta), P23(L2))

    return potential_energy(link1, T01) + potential_energy(link2, T01 * T12) + potential_energy(link3, T01 * T12 * T23)

def main():

    links = obtain_links()
    link1 = links['link1']; link2 = links['link2']; link3 = links['link3']
    K = sum_of_ke(link1, link2, link3)
    P = sum_of_pe(link1, link2, link3)

    L = K - P
    
    #For theta1:
    term1 = diff(L, theta_d1)
    term11 = diff(term1, t)
    term2 = diff(L, theta1)
    tau_1 = simplify(term11 - term2) 
    print("Tau_1 is: ", tau_1)

    #For theta2:
    term1 = diff(L, theta_d2)
    term11 = diff(term1, t)
    term2 = diff(L, theta2)
    tau_2 = simplify(term11 - term2) 
    print("\nTau_2 is: ", tau_2)

    #For theta3:
    term1 = diff(L, theta_d3)
    term11 = diff(term1, t)
    term2 = diff(L, theta3)
    tau_3 = simplify(term11 - term2) 
    tau_3 = simplify(tau_3)
    print("\nTau_3 is: ", tau_3)

    f = open('output.txt', 'w')
    f.write(mlatex(tau_3)+'\n')
    f.write(mlatex(tau_2)+'\n')
    f.write(mlatex(tau_1)+'\n')
    # print("\nDerivative wrt to theta_d1", term1)
    # print()

    # For theta 1:
    # tau_11 = -1 * diff(K, theta1)
    # tau_12 = diff(P, theta1)



if __name__ == "__main__":
    main()