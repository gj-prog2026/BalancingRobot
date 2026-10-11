p = robot_params();
[A,B] = linear_model(p);

Q = diag([1 1 100 1]);      % cost on [x, xdot, theta, thetadot]
R = 1;                      % cost on voltage
K = lqr(A,B,Q,R)            % feedback gain, u = -K*z

Acl = A - B*K;                          % closed-loop system
z0  = [0; 0; deg2rad(5); 0];            % start leaning 5 degrees
t   = linspace(0, 3, 3001);
sys = ss(Acl, zeros(4,1), eye(4), 0);   % no input, just the free response
Z   = initial(sys, z0, t);              % rows = time, columns = states
U   = -(K*Z')';                         % voltage the controller asks for

peak_volts = max(abs(U))
peak_speed = max(abs(Z(:,2)))

figure
subplot(3,1,1); plot(t, rad2deg(Z(:,3))); ylabel('pitch (deg)'); grid on
subplot(3,1,2); plot(t, U); yline(p.V,'r--'); yline(-p.V,'r--'); ylabel('volts'); grid on
subplot(3,1,3); plot(t, Z(:,2)); ylabel('speed (m/s)'); xlabel('time (s)'); grid on