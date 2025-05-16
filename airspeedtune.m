% === Load and map raw CSV data ===
data = readtable('distance_logairspeed5hz.csv');

% Show variable names if unsure
disp(data.Properties.VariableNames)

% Access using actual column names
time = data.Time_s_;                  % Time (s)
u = data.AirspeedInput_m_s_;         % Airspeed Input (m/s)
y = data.DistanceOutput_m_;          % Distance Output (m)
alt = data.Altitude_m_;               % Altitude (m)
airspeed_actual = data.ActualAirspeed_m_s_;  % Actual Airspeed

% Sampling time
Ts = 0.2;

% Create ID data object
id_data = iddata(y, u, Ts);

% === Identify second-order transfer function ===
sys_tf = tfest(id_data, 2, 1);
disp('✅ Identified Transfer Function:');
sys_tf

% === PID Controller Tuning (no d2c needed if already continuous) ===
sys_c = sys_tf;

[C, info] = pidtune(sys_c, 'PID');
disp('✅ Tuned PID Controller:');
C

% === Closed-loop Simulation ===
T_end = time(end);
t = 0:Ts:T_end;

u_sim = interp1(time, u, t, 'previous', 'extrap');
sys_cl = feedback(C * sys_c, 1);
y_sim = lsim(sys_cl, u_sim, t);

% === Plot Results ===
figure;

subplot(3,1,1)
plot(time, u, 'b', 'LineWidth', 1.5);
ylabel('Airspeed Input (m/s)');
title('Step Input');
grid on;

subplot(3,1,2)
plot(time, y, 'k--', 'DisplayName', 'Measured Distance'); hold on;
plot(t, y_sim, 'r', 'LineWidth', 1.5, 'DisplayName', 'Simulated Response');
ylabel('Distance (m)');
title('Distance Output (System ID + PID Response)');
legend;
grid on;

subplot(3,1,3)
plot(time, airspeed_actual, 'g', 'LineWidth', 1.2);
ylabel('Actual Airspeed (m/s)');
xlabel('Time (s)');
title('Actual Airspeed Feedback');
grid on;





