function [t, Z, U] = run_sim(p, K, theta0_deg)
    Ts = p.Ts;  sub = 10;  N = round(3/Ts);
    fun = @(z,u) dynamics_nl(z, u, p);
    z = [0; 0; deg2rad(theta0_deg); 0];
    Z = zeros(N,4);  U = zeros(N,1);
    for k = 1:N
        u = -K*z;                          % controller: once per sample
        u = max(-p.V, min(p.V, u));        % voltage limit
        for s = 1:sub                      % plant: 10 small steps per sample
            z = rk4_step(fun, z, u, Ts/sub);
        end
        Z(k,:) = z';  U(k) = u;
        if abs(z(3)) > deg2rad(40)         % your firmware cut-off
            Z = Z(1:k,:);  U = U(1:k);  break
        end
    end
    t = (1:size(Z,1))' * Ts;
end