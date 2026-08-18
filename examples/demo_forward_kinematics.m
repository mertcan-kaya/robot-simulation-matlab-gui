%% Demo: Forward & Inverse Kinematics (Headless Mode)
% This script demonstrates how to use the +robotics package programmatically
% to perform forward kinematics, transformation chain inspection, and
% closed-loop inverse kinematics for multiple robot models without GUI.

clear; clc; close all;
addpath(fullfile(pwd, '..')); % Ensure project root is on MATLAB path

%% 1. Instantiate Robot Models via RobotFactory
% Model IDs: 1: Franka Emika, 2: UR3, 3: Unitree Z1, 4: Staubli RX160, 5: RX160L, 0: Custom
robot = robotics.models.RobotFactory.create(1); % Franka Emika (7-DoF)
kin = robot.getKinematicParameters(0);          % Standard tool attachment (0: flange, 1: tool)

fprintf('Instantiated robot: %s (%d DoF)\n', robot.Name, kin.n);

%% 2. Evaluate Forward Kinematics (FK)
% Define a test joint configuration [rad]
q_test = [0.0; -pi/6; 0.0; -pi/3; 0.0; pi/2; pi/4];
TI_0 = eye(4); % Base coordinate frame (world)

% Compute full forward transformation chain (4x4x(n+2))
T_chain = robotics.engines.KinematicsEngine.getTransMatrix(...
    TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q_test);

% Extract End-Effector pose (slice n+2)
T_EE = T_chain(:,:,kin.n+2);
p_EE = T_EE(1:3, 4);
R_EE = T_EE(1:3, 1:3);

% Convert rotation matrix to ZYZ Euler angles
eul_ZYZ = robotics.math.getEulerPosVec(R_EE, 1);

fprintf('\nForward Kinematics Results:\n');
fprintf('  End-Effector Position [m]:      X = %.4f, Y = %.4f, Z = %.4f\n', p_EE(1), p_EE(2), p_EE(3));
fprintf('  End-Effector Orientation (ZYZ): phi = %.2f°, theta = %.2f°, psi = %.2f°\n', ...
    rad2deg(eul_ZYZ(1)), rad2deg(eul_ZYZ(2)), rad2deg(eul_ZYZ(3)));

%% 3. Closed-Loop Inverse Kinematics (IK)
% Define target goal pose
tGoal = p_EE + [0.05; -0.05; 0.02]; % Offset target slightly
RGoal = R_EE;

invGeoConfig.robot_model = 1;
invGeoConfig.inv_geo_type = 1;      % 0: Analytic, 1: Numerical DLS, 2: Hybrid
invGeoConfig.TI_0 = TI_0;
invGeoConfig.inv_geo_trn = 1;       % 0: Transpose, 1: Damped Least Squares (DLS)
invGeoConfig.kp_trn = 0.5;
invGeoConfig.kr_trn = 0.5;

q0 = q_test; % Seed configuration
qSolved = robotics.engines.KinematicsEngine.inverseKinematics(robot, invGeoConfig, kin, RGoal, tGoal, q0);

% Verify solved pose via forward kinematics
T_solved_chain = robotics.engines.KinematicsEngine.getTransMatrix(...
    TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, qSolved);
p_solved = T_solved_chain(1:3, 4, kin.n+2);

pos_error_mm = norm(tGoal - p_solved) * 1000;
fprintf('\nInverse Kinematics Solution:\n');
fprintf('  Target position [m]:   [%.4f, %.4f, %.4f]\n', tGoal(1), tGoal(2), tGoal(3));
fprintf('  Solved position [m]:   [%.4f, %.4f, %.4f]\n', p_solved(1), p_solved(2), p_solved(3));
fprintf('  Position Error:        %.3f mm\n', pos_error_mm);

%% 4. Plot 3D Kinematic Skeleton
figure('Name', 'Robot Kinematic Chain', 'Color', 'w');
hold on; grid on; axis equal;
view(45, 30);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('%s - 3D Forward Kinematics', robot.Name));

% Extract joint positions from transformation chain
joint_positions = squeeze(T_chain(1:3, 4, :));

% Draw kinematic links
plot3(joint_positions(1,:), joint_positions(2,:), joint_positions(3,:), ...
    '-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', [0.2 0.6 0.9], 'Color', [0.1 0.3 0.7]);

% Highlight base and end-effector
plot3(joint_positions(1,1), joint_positions(2,1), joint_positions(3,1), ...
    's', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'Color', 'k');
plot3(p_EE(1), p_EE(2), p_EE(3), ...
    'p', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'Color', 'r');

legend('Links', 'Base Frame', 'End-Effector', 'Location', 'best');
