%% Demo: Trajectory Planning & Dynamic Simulation (Headless Mode)
% This script demonstrates how to plan smooth joint-space trajectories,
% compute minimum required motion durations, and simulate inverse/forward
% dynamics using the +robotics package in headless mode.

clear; clc; close all;
addpath(fullfile(pwd, '..')); % Ensure project root is on MATLAB path

%% 1. Configure Robot Model & Physics
robot = robotics.models.RobotFactory.create(2); % Universal Robots UR3 (6-DoF)
kin = robot.getKinematicParameters(0);
dyn = robot.getInertialParameters();

fprintf('Simulating Robot: %s (%d DoF)\n', robot.Name, kin.n);

%% 2. Define Trajectory Endpoints & Compute Time
q_initial = zeros(kin.n, 1);
q_final   = [deg2rad(45); deg2rad(-30); deg2rad(60); deg2rad(-90); deg2rad(45); deg2rad(30)];

% Trajectory Profile: 1: Linear, 2: Cubic, 3: Quintic, 4: Trapezoidal
trj_profile = 3; % Smooth C^2 quintic polynomial
prcnt_velocity = 80; % 80% maximum joint speed

% Dynamically compute minimum safe execution time
tfin_trj = robotics.engines.TrajectoryEngine.computeTime(...
    q_initial, q_final, prcnt_velocity, trj_profile, kin.q_velLim, kin.q_accLim);

fprintf('Trajectory Profile: Quintic Polynomial\n');
fprintf('Calculated Execution Duration: %.3f seconds\n', tfin_trj);

%% 3. Generate Trajectory Time Series
dt = 0.001; % 1 ms sampling step (1 kHz)
time_vec = 0:dt:tfin_trj;
N_steps = length(time_vec);

trjConfig.tstp = dt;
trjConfig.tfin_trj = tfin_trj;
trjConfig.trj_profile = trj_profile;

q_des   = zeros(kin.n, N_steps);
qd_des  = zeros(kin.n, N_steps);
qdd_des = zeros(kin.n, N_steps);
tau_inv = zeros(kin.n, N_steps);
g_vector = [0; 0; -9.81];

for k = 1:N_steps
    % Evaluate analytical trajectory at step k-1
    [q_k, qd_k, qdd_k] = robotics.engines.TrajectoryEngine.generateTrajectory(...
        trjConfig, kin, q_initial, q_final, k - 1);
    
    q_des(:, k)   = q_k;
    qd_des(:, k)  = qd_k;
    qdd_des(:, k) = qdd_k;
    
    % Evaluate inverse dynamics (RNEA/MNEA) to find required feedforward torques
    tau_inv(:, k) = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(...
        kin, q_k, qd_k, qd_k, qdd_k, g_vector, dyn.pj_j);
end

%% 4. Plot Trajectory and Torque Profiles
figure('Name', 'Trajectory & Dynamic Torques', 'Color', 'w', 'Position', [100 100 900 650]);

subplot(3,1,1);
plot(time_vec, q_des, 'LineWidth', 1.5);
grid on; ylabel('Position [rad]');
title(sprintf('%s - Joint Space Trajectory & Dynamic Control', robot.Name));
legend(arrayfun(@(j) sprintf('Joint %d', j), 1:kin.n, 'UniformOutput', false), 'Location', 'eastoutside');

subplot(3,1,2);
plot(time_vec, qd_des, 'LineWidth', 1.5);
grid on; ylabel('Velocity [rad/s]');

subplot(3,1,3);
plot(time_vec, tau_inv, 'LineWidth', 1.5);
grid on; xlabel('Time [s]'); ylabel('Torque [N\cdotm]');

fprintf('Simulation complete. Plotted %d trajectory steps.\n', N_steps);
