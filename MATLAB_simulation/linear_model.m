function [A, B] = linear_model(p)
    Meq = p.m_w + p.J_w/p.r^2;      % wheel's equivalent mass
    E   = [p.m+Meq, p.m*p.l         
           p.m*p.l, p.I+p.m*p.l^2]; % mass matrix of [xdd; thetadd]
    d   = [1/p.r; -1];              % how torque tau enters each equation
    a   = 2*p.Kt/p.R;               % total torque per volt

    q   = E \ d;                    % acceleration per unit torque
    w   = E \ [0; p.m*p.g*p.l];     % acceleration per unit lean (gravity)

    % tau = a*u + c_xd*xdot + c_thd*theadot
    c_xd = -a*p.Ke/p.r;
    c_thd = a*p.Ke;

    A = [0, 1,         0,   0
         0, q(1)*c_xd, w(1), q(1)*c_thd
         0, 0,         0,   1
         0, q(2)*c_xd, w(2), q(2)*c_thd];
    B = [0; q(1)*a; 0; q(2)*a];
end