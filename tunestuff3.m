% Load data
data = readtable('distance_log5hz.csv');

% Extract relevant signals
time = data.Time_s_;
u = data.TargetDistance_m_;   % Input: Commanded distance
y = data.Distance_m_;         % Output: Measured distance

% Ignore transient phase (e.g., first 30 seconds)
start_time = 300;
valid_idx = time <= start_time;

% Apply selection
time = time(valid_idx);
u = u(valid_idx);
y = y(valid_idx);

% Define sampling time (5 Hz data)
Ts = 0.2;

% Optional: Apply low-pass filtering to reduce noise (uncomment if needed)
% y = smoothdata(y, 'movmean', 5);
% u = smoothdata(u, 'movmean', 5);

% Create IDDATA object for system identification
sys_data = iddata(y, u, Ts);

% Plot command vs. actual response
figure;
plot(time, u, 'b--', 'LineWidth', 1.2); hold on;
plot(time, y, 'r-', 'LineWidth', 1.2);
legend('Target Distance', 'Actual Distance', 'Location', 'Best');
xlabel('Time [s]');
ylabel('Distance [m]');
title('Target vs Actual Distance (Post Transient)');
grid on;

% Estimate transfer function (2 poles, 1 zero)
sys_tf = tfest(sys_data, 2, 1);

% Display estimated transfer function
disp('Estimated Transfer Function:');
disp(sys_tf);

% Compare model to measured data
figure;
compare(sys_data, sys_tf);
title('Model Validation: Measured vs Estimated Output');

% Tune PID controller based on identified plant
pid_controller = pidtune(sys_tf, 'PID');

% Display tuned PID gains
disp('Tuned PID Gains:');
disp(pid_controller);

% Simulate step response of closed-loop system
T = feedback(pid_controller * sys_tf, 1);
figure;
step(T);
title('Closed-loop Step Response with Tuned PID');
xlabel('Time [s]');
ylabel('Distance [m]');
grid on;

% Define ARX model orders: na (output), nb (input), nk (delay)
na = 2; nb = 2; nc = 2; nk = 1;
sys_armax = armax(sys_data, [na nb nc nk]);
disp('ARMAX model:');
disp(sys_armax);

% Display ARX model
disp('Estimated ARX Model:');
disp(sys_arx);

sys_ss = ssest(sys_data, 2);  % 2 states, adjust as needed
disp('State-space model:');
disp(sys_ss);

% Tune PID controller based on identified plant
pid_controller2 = pidtune(sys_ss, 'PID');

% Display tuned PID gains
disp('Tuned PID Gains SS:');
disp(pid_controller2);

% Compare ARX model to measured data
figure;
compare(sys_data, sys_tf, sys_arx, sys_ss);
legend('Measured Output', 'TF Model', 'ARX Model', 'SS Model');
title('Model Comparison: Transfer Function vs ARX vs SS');
