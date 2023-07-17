import numpy as np
import sympy
from sympy import symbols
from sympy import cos, sin
from Transformation import R01, R12, R23, R34, P01, P12, P23, P34, I1, I2, I3
from sympy import Matrix, Function, diff
from sympy import simplify

t = symbols('t')
theta1 = Function('theta1')(t); theta2 = Function('theta2')(t); theta3 = Function('theta3')(t)
theta_d1 = diff(theta1, t); theta_d2 = diff(theta2, t); theta_d3 = diff(theta3, t)
theta_dd1 = diff(theta_d1, t); theta_dd2 = diff(theta_d2, t); theta_dd3 = diff(theta_d3, t)
L1, L2, L3 = symbols('L1 L2 L3')
m1, m2, m3, I1xx, I1yy, I1zz, I2xx, I2yy, I2zz,  I3xx, I3yy, I3zz = symbols('m1 m2 m3 I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz')
g = symbols('g')

Inertia = {
    'I1xx': I1xx, 
    'I1yy': I1yy,
    'I1zz': I1zz,
    'I2xx': I2xx, 
    'I2yy': I2yy,
    'I2zz': I2zz,
    'I3xx': I3xx, 
    'I3yy': I3yy,
    'I3zz': I3zz,
}
    
mass = {
    'mass_1': m1,
    'mass_2': m2,
    'mass_3': m3,
    'I1': I1(Inertia),
    'I2': I2(Inertia),
    'I3': I3(Inertia)
}
    
theta = {
    '1': theta1,
    '2': theta2,
    '3': theta3
}
    
theta_dot = {
    '1': theta_d1,
    '2': theta_d2,
    '3': theta_d3
}
    
theta_ddot = {
    '1': theta_dd1,
    '2': theta_dd2, 
    '3': theta_dd3
}
    

functions = {
    'R01': R01(theta),
    'R12': R12(theta),
    'R23': R23(theta),
    'R34': R34(theta),
    'P01': P01(L1),
    'P12': P12(),
    'P23': P23(L2),
    'P34': P34(L3),
    'PC0': Matrix([0, 0, -1 * L1/2]),
    'PC1': Matrix([L2/2,   0,    0]),
    'PC2': Matrix([L3/2,   0,    0])
}

def outward_iteration(params):

    old = params['link']
    new = old + 1

    R = functions['R'+str(old)+str(new)].T
    P = functions['P'+str(old)+str(new)]
    P_C = functions['PC'+str(old)]
    
    v_n = R * (params['v'] + params['w'].cross(P))
    w_n = R * params['w'] + Matrix([0, 0, theta_dot[str(new)]])

    w_dot_n = R * params['w_dot'] + (R * params['w']).cross(Matrix([0, 0, theta_dot[str(new)]])) + Matrix([0, 0, theta_ddot[str(new)]])
    v_dot_n = R * (params['w'].cross(P) + w_n.cross(w_n.cross(P)) + params['v_dot'])

    v_c_n = w_n.cross(P_C)  + v_n
    v_c_dot_n = w_dot_n.cross( P_C) + w_n.cross(w_n.cross(P_C)) + v_dot_n

    F_n = mass['mass_'+str(new)] * v_c_dot_n
    N_n = mass['I'+str(new)] * w_dot_n + w_n.cross(mass['I'+ str(new)]  * w_n)

    params_new = {
        'link': new,
        'v': v_n,
        'w': w_n,
        'w_dot': w_dot_n,
        'v_dot': v_dot_n,
        'v_c': v_c_n, 
        'v_c_dot': v_c_dot_n,
        'F': F_n,
        'N': N_n
    }

    params_new = simplify_expressions(params_new)

    return params_new

# def inward_iteration(params):

def simplify_expressions(params):

    params['v'] = simplify(params['v'])
    params['w'] = simplify(params['w'])
    params['w_dot'] = simplify(params['w_dot'])
    params['v_dot'] = simplify(params['v_dot'])
    params['v_c'] = simplify(params['v_c'])
    params['v_c_dot'] = simplify(params['v_c_dot'])
    params['F'] = simplify(params['F'])
    params['N'] = simplify(params['N'])

    return params

def inward_iterations(params, forces):

    new = params['link']
    old = new - 1

    R = functions['R'+str(old)+str(new)]
    P_C = functions['PC'+str(old)]
    P = functions['P'+str(old)+str(new)]
    
    f_o = simplify(R * forces['f'] + params['F'])
    n_o = simplify(params['N'] + R * forces['n'] + P_C.cross(params['F']) + P.cross(R * forces['f']))

    tau = simplify(n_o[2])

    forces = {
        'f': f_o,
        'n': n_o,
        'tau': tau
    }

    return forces

def obtain_links():
    w_0 = Matrix([0, 0, 0])
    v_0 = Matrix([0, 0, 0])
    v_dot_0 = Matrix([0, 0, g])
    w_dot_0 = Matrix([0, 0, 0])

    link0 = {
        'link': 0,
        'v': v_0,
        'w': w_0,
        'v_dot': v_dot_0,
        'w_dot': w_dot_0,
    }

    link1 = outward_iteration(link0)
    link2 = outward_iteration(link1)
    link3 = outward_iteration(link2)

    return {'link1': link1, 'link2': link2, 'link3': link3}
    
def main():

    w_0 = Matrix([0, 0, 0])
    v_0 = Matrix([0, 0, 0])
    v_dot_0 = Matrix([0, 0, g])
    w_dot_0 = Matrix([0, 0, 0])

    link0 = {
        'link': 0,
        'v': v_0,
        'w': w_0,
        'v_dot': v_dot_0,
        'w_dot': w_dot_0,
    }

    link1 = outward_iteration(link0)
    link2 = outward_iteration(link1)
    link3 = outward_iteration(link2)
    # link4 = outward_iteration(link3)

    forces = {
        'n': Matrix([0, 0, 0]),
        'f': Matrix([0, 0, 0])
    }

    force_3 = inward_iterations(link3, forces)
    print('\nTau_3 is: ', force_3['tau'])
    force_2 = inward_iterations(link2, force_3)
    print('\nTau_2 is: ', force_2['tau'])
    force_1 = inward_iterations(link1, force_2)
    print('\nTau_1 is: ', force_1['tau'])


if __name__ == "__main__":

    main()



