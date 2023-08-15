syms I31_1 I31_2 I31_3 I32_1 I32_2 I32_3 I12_1 I12_2 I12_3 I13_1 I13_2 I13_3 I21_1 I21_2 I21_3 I23_1 I23_2 I23_3 g q1 q2 q3 qdd1 qdd2 qdd3 qd1 qd2 qd3

equations = urdf2eom("./URDFs/manipulator.urdf")

eqn_wo = subs(equations, [I31_1 I31_2 I31_3 I32_1 I32_2 I32_3 I12_1 I12_2 I12_3 I13_1 I13_2 I13_3 I21_1 I21_2 I21_3 I23_1 I23_2 I23_3], [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0])

M_sym = collect(eqn_wo, [qdd1, qdd2, qdd3]);
M_matrix = equationsToMatrix(M_sym, [qdd1, qdd2, qdd3])

M_eqn = M_matrix * [qdd1 qdd2 qdd3].'

eqn_wo_M = simplify(eqn_wo - M_eqn)

g_eqn = collect(eqn_wo_M, g)