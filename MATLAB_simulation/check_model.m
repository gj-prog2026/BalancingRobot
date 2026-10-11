p = struct('m',0.323, 'l',0.0735, 'I',4.0e-3, ...
           'm_w',0.032, 'r',0.0215, 'J_w',7.4e-6, ...
           'R',11.6, 'Ke',0.115, 'Kt',0.115, 'g',9.81);

[A,B] = linear_model(p)

eigenvalues = eig(A)
controllability_rank = rank(ctrb(A,B))

p0 = p;  p0.Kt = 0;                 % motors off
motors_off = eig(linear_model(p0))