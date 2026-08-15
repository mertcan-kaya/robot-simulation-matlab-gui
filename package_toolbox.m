% package_toolbox.m
disp('Configuring Toolbox Options...');

% Fetch version from single source of truth
ver = robotics.version();
disp(['Toolbox Version: ', ver]);

opts = matlab.addons.toolbox.ToolboxOptions(pwd, 'robot-sim-gui');
opts.ToolboxName = 'Robot Simulation GUI';
% MATLAB requires numeric ToolboxVersion (e.g. 1.0.0)
numVer = regexp(ver, '^\d+(\.\d+)+', 'match', 'once');
if isempty(numVer)
    numVer = '1.0.0';
end
opts.ToolboxVersion = numVer;
opts.AuthorName = 'Mertcan Kaya';
opts.AuthorEmail = '';
opts.Summary = 'A MATLAB App Designer GUI for simulating and controlling robotic manipulators.';
opts.Description = 'This toolbox provides a full 3D simulation environment and kinematics/dynamics engines for various robots including Franka Emika, UR3, and Unitree Z1.';

if ~exist('releases', 'dir')
    mkdir('releases');
end
opts.OutputFile = fullfile(pwd, 'releases', 'Robot_Simulation_GUI.mltbx');

% Add subfolders to the MATLAB path upon installation
opts.ToolboxMatlabPath = {pwd, fullfile(pwd, 'meshes')};

disp('Packaging Toolbox... This may take a moment.');
matlab.addons.toolbox.packageToolbox(opts);
disp(['Toolbox successfully packaged at: ', opts.OutputFile]);
