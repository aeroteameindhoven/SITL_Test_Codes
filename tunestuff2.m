clc;
clear;

% Load CSV
data = readtable('distance_log5hz.csv');

% Extract variables using actual column names
t = data.("Time (s)");
ref = data.("Target Distance (m)");
err = data.("Distance Error (m)");
u = data.("Airspeed Command (m/s)");

% Actual distance (likely mislabeled as 'Actual Airspeed (m/s), Distance (m)')
y = ref + err;  % Assuming error = actual - target


% Compute actual distance (output)
y = ref + err;

% Normalize time
t = t - t(1);
Ts = mean(diff(t));

% Create IDDATA object
sys_data = iddata(y, u, Ts);

% Plot input-output
figure;
plot(sys_data);
title('UAV Airspeed Command → Actual Distance');
xlabel('Time (s)');

% System Identification
np = 2; nz = 1;
sys_tf = tfest(sys_data, np, nz);

disp('--- Identified Transfer Function ---');
sys_tf

% Validate fit
figure;
compare(sys_data, sys_tf);
title('Model Validation');

% PID Tuning
[C, info] = pidtune(sys_tf, 'PID');
fprintf('\n--- PID Gains ---\nKp = %.4f\nKi = %.4f\nKd = %.4f\n', C.Kp, C.Ki, C.Kd);

% Simulate closed-loop
T = feedback(C * sys_tf, 1);
figure;
step(T);
title('Closed-Loop Step Response')

