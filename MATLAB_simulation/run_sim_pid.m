function [t, Z, PWM, t_fall] = run_sim_pid(p, theta0_deg, T)
    Ts = p.Ts;  sub = 10;  N = round(T/Ts);
    fun = @(z,u) dynamics_nl(z, u, p);
    z = [0; 0; deg2rad(theta0_deg); 0];
    integ = 0;  prev_err = 0;
    Z = zeros(N,4);  PWM = zeros(N,1);  t_fall = NaN;
    for k = 1:N
        % --- same maths as PID_Update ---
        err   = rad2deg(z(3));             % degrees, positive leaning forward
        integ = max(-200, min(200, integ + err*Ts));
        deriv = (err - prev_err)/Ts;  prev_err = err;
        out   = p.Kp*err + p.Ki*integ + p.Kd*deriv;
        pwm   = fix(max(-999, min(999, out)));   % int16 cast truncates toward zero

        % --- your deadband floor ---
        if p.pwm_floor > 0
            if pwm > 0 && pwm <  p.pwm_floor, pwm =  p.pwm_floor; end
            if pwm < 0 && pwm > -p.pwm_floor, pwm = -p.pwm_floor; end
        end

        u = pwm/999 * p.V;                 % PWM counts -> volts
        for s = 1:sub
            z = rk4_step(fun, z, u, Ts/sub);
        end
        Z(k,:) = z';  PWM(k) = pwm;
        if abs(z(3)) > deg2rad(40)
            t_fall = k*Ts;  Z = Z(1:k,:);  PWM = PWM(1:k);  break
        end
    end
    t = (1:size(Z,1))' * Ts;
end