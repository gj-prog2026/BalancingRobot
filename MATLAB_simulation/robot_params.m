function p = robot_params()
    p.g   = 9.81;
    p.m   = 0.323;    % body mass (kg), measured
    p.l   = 0.0735;   % axle to CoM (m), measured
    p.I   = 4.0e-3;   % body inertia (kg m^2), swing test, provisional
    p.m_w = 0.032;    % both wheels (kg), measured
    p.r   = 0.0215;   % wheel radius (m), measured
    p.J_w = 7.4e-6;   % both wheels (kg m^2), disc estimate
    p.R   = 11.6;     % winding resistance (ohm), measured
    p.Ke  = 0.115;    % V s/rad, from the no-load run (upper bound 0.120)
    p.Kt  = 0.115;    % N m/A, assumed equal to Ke
    p.V   = 5.7;      % supply (V), measured at the battery
    p.Ts  = 0.01;     % control period (s)
    p.Kp = 20;  p.Ki = 0;  p.Kd = 0;    % the firmware gains
    p.pwm_floor = 0;                    % 0 = no floor; try 120 and 155
end