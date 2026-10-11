p = robot_params();
[A,B] = linear_model(p);
K = lqr(A,B,diag([1 1 100 1]),1);

[t,Z,U] = run_sim(p, K, 5);
peak_volts      = max(abs(U))
final_pitch_deg = rad2deg(Z(end,3))

figure; hold on
for th = [5 15 25 30]
    [t,Z,~] = run_sim(p, K, th);
    plot(t, rad2deg(Z(:,3)), 'DisplayName', sprintf('%d deg', th));
end
legend; grid on; xlabel('time (s)'); ylabel('pitch (deg)')