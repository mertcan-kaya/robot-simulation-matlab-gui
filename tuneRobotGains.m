% TUNEROBOTGAINS Automated tuning and gain inspection script
% Calculates model-based optimal gains for each robot manipulator across
% Joint PID, Joint IDC, Task PID, and Task IDC controllers.

robotModels = {
    'Franka Emika (7 DoF)', 1;
    'Universal Robots UR3 (6 DoF)', 2;
    'Unitree Z1 (6 DoF)', 3;
    'Staubli RX160 (6 DoF)', 4;
    'Staubli RX160L (6 DoF)', 5
};

fprintf('\n========================================================================\n');
fprintf('                ROBOT MANIPULATOR OPTIMAL GAIN TUNING                   \n');
fprintf('========================================================================\n\n');

for k = 1:size(robotModels, 1)
    name = robotModels{k, 1};
    id = robotModels{k, 2};
    
    robot = robotics.models.RobotFactory.create(id);
    ctr = robotics.engines.GainTuningEngine.computeOptimalGains(robot);
    
    fprintf('>>> Robot: %s (t_cyc = %.3f s)\n', name, ctr.tcyc);
    fprintf('------------------------------------------------------------------------\n');
    
    % Joint PID Gains
    fprintf('  Joint Space PID Gains (Scaled by Inertia Tensor):\n');
    fprintf('    Kp: [%s]\n', strjoin(string(ctr.Kp_jnt_pid), ', '));
    fprintf('    Ki: [%s]\n', strjoin(string(ctr.Ki_jnt_pid), ', '));
    fprintf('    Kd: [%s]\n', strjoin(string(ctr.Kd_jnt_pid), ', '));
    
    % Joint IDC Gains
    fprintf('  Joint Space IDC Gains (Critically Damped):\n');
    fprintf('    Kp: [%s]\n', strjoin(string(ctr.Kp_jnt_idc), ', '));
    fprintf('    Ki: [%s]\n', strjoin(string(ctr.Ki_jnt_idc), ', '));
    fprintf('    Kd: [%s]\n', strjoin(string(ctr.Kd_jnt_idc), ', '));
    
    % Task PID Gains
    fprintf('  Task Space PID Gains (Cartesian Impedance):\n');
    fprintf('    Kp: [%s]\n', strjoin(string(ctr.Kp_tsk_pid), ', '));
    fprintf('    Ki: [%s]\n', strjoin(string(ctr.Ki_tsk_pid), ', '));
    fprintf('    Kd: [%s]\n', strjoin(string(ctr.Kd_tsk_pid), ', '));
    
    % Task IDC Gains
    fprintf('  Task Space IDC Gains (Decoupled Cartesian Acceleration):\n');
    fprintf('    Kp: [%s]\n', strjoin(string(ctr.Kp_tsk_idc), ', '));
    fprintf('    Ki: [%s]\n', strjoin(string(ctr.Ki_tsk_idc), ', '));
    fprintf('    Kd: [%s]\n', strjoin(string(ctr.Kd_tsk_idc), ', '));
    fprintf('\n');
end

fprintf('========================================================================\n');
fprintf('Optimal gain calculations completed successfully.\n');
