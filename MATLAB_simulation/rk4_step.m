function z = rk4_step(fun, z, u, dt)
    k1 = fun(z, u);
    k2 = fun(z + dt/2*k1, u);
    k3 = fun(z + dt/2*k2, u);
    k4 = fun(z + dt*k3, u);
    z  = z + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end