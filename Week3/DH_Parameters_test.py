from sympy import symbols, cos, sin, Matrix, pprint

theta, d, a, alpha = symbols('theta d a alpha')

T_Rz = Matrix([
    [cos(theta), -sin(theta), 0, 0],
    [sin(theta),  cos(theta), 0, 0],
    [0,           0,          1, 0],
    [0,           0,          0, 1]])

T_zd = Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, d],
    [0, 0, 0, 1]])

T_xa = Matrix([
    [1, 0, 0, a],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]])

T_Rx = Matrix([
    [1, 0,           0,          0],
    [0, cos(alpha), -sin(alpha), 0],
    [0, sin(alpha),  cos(alpha), 0],
    [0, 0,           0,          1]])

T = T_Rz * T_zd * T_xa * T_Rx
pprint(T)