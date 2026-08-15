classdef TestTrjGeneration < matlab.unittest.TestCase
    % Tests for the trajectory generation module
    
    methods(Test)
        function testTrajectoryOutput(testCase)
            trjConfig.tstp = 0.01;
            trjConfig.tfin_trj = 1.0;
            trjConfig.trj_profile = 2; % Cubic interpolation
            
            robot = robotics.models.RobotFactory.create(2); % UR3
            kin = robot.getKinematicParameters(1);
            
            ini_q = zeros(kin.n, 1);
            fin_q = ones(kin.n, 1);
            
            k = 50; % Halfway (0.5s)
            
            [q_pos, q_vel, q_acc] = robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, kin, ini_q, fin_q, k);
            
            testCase.verifySize(q_pos, [kin.n, 1]);
            testCase.verifySize(q_vel, [kin.n, 1]);
            testCase.verifySize(q_acc, [kin.n, 1]);
            
            % At t = 0.5s for a 1.0s cubic trajectory, position should be exactly 0.5
            testCase.verifyEqual(q_pos(1), 0.5, 'RelTol', 1e-4);
        end
        
        function testNoInterpolation(testCase)
            trjConfig.tstp = 0.01;
            trjConfig.tfin_trj = 1.0;
            trjConfig.trj_profile = 0; % No interpolation
            
            robot = robotics.models.RobotFactory.create(1); % Franka
            kin = robot.getKinematicParameters(1);
            
            ini_q = zeros(kin.n, 1);
            fin_q = ones(kin.n, 1);
            
            k = 50;
            
            [q_pos, q_vel, q_acc] = robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, kin, ini_q, fin_q, k);
            
            % Should just return final position with 0 velocity and acceleration
            testCase.verifyEqual(q_pos(1), 1.0);
            testCase.verifyEqual(q_vel(1), 0.0);
            testCase.verifyEqual(q_acc(1), 0.0);
        end
    end
end

