classdef TestKinematics < matlab.unittest.TestCase
    % TESTKINEMATICS Unit tests for forward and inverse kinematics in +robotics/+engines.

    methods (Test)

        function testForwardKinematicsTransformChains(testCase)
            % Test getTransMatrix and getRelativeTransMatrix for multiple robots
            robot_ids = [1, 2, 3, 4, 5]; % Franka, UR3, Unitree, Staubli RX160, Staubli RX160L

            for id = robot_ids
                robot = robotics.models.RobotFactory.create(id);
                kin = robot.getKinematicParameters(0);
                q = zeros(kin.n, 1);
                TI_0 = eye(4);

                % Relative transformations
                TR_i = robotics.engines.KinematicsEngine.getRelativeTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q);
                testCase.verifySize(TR_i, [4, 4, kin.n + 2]);

                % Cumulative transformations
                TI_i = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q);
                testCase.verifySize(TI_i, [4, 4, kin.n + 2]);

                % Verify base frame is identity
                testCase.verifyEqual(TI_i(:,:,1), eye(4), 'AbsTol', 1e-12);

                % Verify chain rule: TI_i(:,:,j+1) == TI_i(:,:,j) * TR_i(:,:,j+1)
                for j = 1:kin.n+1
                    T_expected = TI_i(:,:,j) * TR_i(:,:,j+1);
                    testCase.verifyEqual(TI_i(:,:,j+1), T_expected, 'AbsTol', 1e-10);
                end
            end
        end

        function testClosedLoopInverseKinematics(testCase)
            % Test that forward kinematics of solved qDes matches commanded target pose
            robot = robotics.models.RobotFactory.create(2); % UR3
            kin = robot.getKinematicParameters(0);

            % Choose a reachable configuration
            q_target = [0.2; -0.4; 0.6; -0.8; 0.3; 0.5];
            TI_0 = eye(4);

            T_target = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q_target);
            RGoal = T_target(1:3, 1:3, kin.n+2);
            tGoal = T_target(1:3, 4, kin.n+2);

            invGeoConfig.robot_model = 2;
            invGeoConfig.inv_geo_type = 2; % Hybrid (analytic with numeric fallback)
            invGeoConfig.TI_0 = TI_0;
            invGeoConfig.inv_geo_trn = 0;
            invGeoConfig.kp_inv = 0.23;
            invGeoConfig.kr_inv = 0.015;

            q0 = zeros(kin.n, 1);
            qDes = robotics.engines.KinematicsEngine.inverseKinematics(robot, invGeoConfig, kin, RGoal, tGoal, q0);

            % Evaluate forward kinematics at solved qDes
            T_solved = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, qDes);
            t_solved = T_solved(1:3, 4, kin.n+2);
            R_solved = T_solved(1:3, 1:3, kin.n+2);

            % Position error should be < 1 mm
            testCase.verifyEqual(t_solved, tGoal, 'AbsTol', 1e-3);
            % Rotation matrix Frobenius norm error
            testCase.verifyEqual(norm(R_solved - RGoal, 'fro'), 0.0, 'AbsTol', 1e-2);
        end

    end
end
