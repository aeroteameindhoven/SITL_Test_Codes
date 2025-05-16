data = readtable("stepinput_throttle_ax.csv");
t = data.time;
u = data.throttle;
y = data.ax;

sys_data = iddata(y, u, mean(diff(t)));
sys = tfest(sys_data, 2, 1);  % 2 poles, 1 zero
bode(sys)
[C, info] = pidtune(sys, 'PID');  % or 'PI', 'PD', 'P'
% Show PID values
fprintf("Tuned Gains:\nKp = %.4f\nKi = %.4f\nKd = %.4f\n", C.Kp, C.Ki, C.Kd);

