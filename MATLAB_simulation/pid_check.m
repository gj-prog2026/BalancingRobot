p = robot_params();
p.Kp = 20;  p.Ki = 0;  p.Kd = 0;
for fl = [0 120 155]
    p.pwm_floor = fl;
    [t,Z,PWM,t_fall] = run_sim_pid(p, 3, 30);   % start 3 deg, run up to 30 s
    fprintf('floor %3d: fell at %.2f s\n', fl, t_fall);
end