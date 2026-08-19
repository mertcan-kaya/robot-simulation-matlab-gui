classdef TestTrjGeneration < matlab.unittest.TestCase
    % Tests for the trajectory generation module
    
    methods(Test)
        function testCubicTrajectory(testCase)
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
            testCase.verifyEqual(q_pos(1), 0.5, 'RelTol', 1e-4);
        end

        function testQuinticTrajectory(testCase)
            trjConfig.tstp = 0.01;
            trjConfig.tfin_trj = 1.0;
            trjConfig.trj_profile = 3; % Quintic interpolation
            
            robot = robotics.models.RobotFactory.create(1); % Franka
            kin = robot.getKinematicParameters(1);
            
            ini_q = zeros(kin.n, 1);
            fin_q = ones(kin.n, 1);
            
            k = 50; % Halfway (0.5s)
            [q_pos, q_vel, q_acc] = robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, kin, ini_q, fin_q, k);
            
            testCase.verifySize(q_pos, [kin.n, 1]);
            testCase.verifyEqual(q_pos(1), 0.5, 'RelTol', 1e-4);
        end

        function testLinearAndTrapezoidalTrajectory(testCase)
            trjConfig.tstp = 0.01;
            trjConfig.tfin_trj = 1.0;
            
            robot = robotics.models.RobotFactory.create(2);
            kin = robot.getKinematicParameters(1);
            ini_q = zeros(kin.n, 1);
            fin_q = ones(kin.n, 1);
            
            % Linear
            trjConfig.trj_profile = 1;
            [q_pos_lin, ~, ~] = robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, kin, ini_q, fin_q, 50);
            testCase.verifyEqual(q_pos_lin(1), 0.5, 'RelTol', 1e-4);

            % Trapezoidal
            trjConfig.trj_profile = 4;
            [q_pos_trap, ~, ~] = robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, kin, ini_q, fin_q, 50);
            testCase.verifyEqual(q_pos_trap(1), 0.5, 'RelTol', 1e-4);
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

        function testTaskSpaceTrajectory(testCase)
            trjConfig.tstp = 0.01;
            trjConfig.tfin_trj = 2.0;
            trjConfig.trj_profile = 3; % Quintic

            robot = robotics.models.RobotFactory.create(2); % UR3
            kin = robot.getKinematicParameters(1);
            invConfig.inv_geo_type = 2;
            invConfig.robot_model = 2;
            invConfig.TI_0 = eye(4);
            invConfig.inv_geo_trn = 0;
            invConfig.kp_inv = 0.23;
            invConfig.kr_inv = 0.015;
            invConfig.kp_trn = 0.003;
            invConfig.kr_trn = 0.0001;

            ini_x = [0.2; 0.1; 0.3; 0.0; 0.0; 0.0];
            fin_x = [0.3; 0.2; 0.4; 0.2; 0.1; 0.0];
            q_prev = zeros(kin.n, 1);

            k = 100; % Halfway (1.0s)
            [q_pos, q_vel, q_acc, x_pos, x_vel, x_acc] = ...
                robotics.engines.TrajectoryEngine.generateTaskSpaceTrajectory(...
                    trjConfig, robot, invConfig, kin, ini_x, fin_x, q_prev, k, 1);

            testCase.verifySize(q_pos, [kin.n, 1]);
            testCase.verifySize(q_vel, [kin.n, 1]);
            testCase.verifySize(q_acc, [kin.n, 1]);
            testCase.verifySize(x_pos, [6, 1]);
            testCase.verifySize(x_vel, [6, 1]);
            testCase.verifySize(x_acc, [6, 1]);

            % Cartesian midpoint verification
            testCase.verifyEqual(x_pos(1), 0.25, 'RelTol', 1e-3);
            testCase.verifyEqual(x_pos(2), 0.15, 'RelTol', 1e-3);
            testCase.verifyEqual(x_pos(3), 0.35, 'RelTol', 1e-3);
        end

        function testTaskSpaceTiming(testCase)
            ini_x = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
            fin_x = [0.5; 0.5; 0.5; 0.2; 0.2; 0.2];
            prcnt = 0.5; % 50% speed

            tf_quintic = robotics.engines.TrajectoryEngine.computeTaskSpaceTime(ini_x, fin_x, prcnt, 3);
            testCase.verifyGreaterThan(tf_quintic, 0);

            tf_none = robotics.engines.TrajectoryEngine.computeTaskSpaceTime(ini_x, fin_x, prcnt, 0);
            testCase.verifyEqual(tf_none, 0);
        end
    end
end

