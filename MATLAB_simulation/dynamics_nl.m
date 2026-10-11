function zd = dynamics_nl(z, u, p)
    xd = z(2);  th = z(3);  thd = z(4);
    Meq = p.m_w + p.J_w/p.r^2;
    a   = 2*p.Kt/p.R;
    tau = a*(u - p.Ke*(xd/p.r - thd));       % total motor torque
    E   = [p.m+Meq,          p.m*p.l*cos(th);
           p.m*p.l*cos(th),  p.I+p.m*p.l^2];
    rhs = [tau/p.r + p.m*p.l*thd^2*sin(th);
           p.m*p.g*p.l*sin(th) - tau];
    acc = E \ rhs;
    zd  = [xd; acc(1); thd; acc(2)];
end