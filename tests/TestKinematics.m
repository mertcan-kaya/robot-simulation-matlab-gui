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
            kin = robot.getKinematicParameters(1); % with standard tool attachment

            % Choose a reachable configuration within UR3 analytic domain
            q_target = [0.1; -0.2; 0.3; -0.4; 0.5; 0.6];
            TI_0 = eye(4);

            T_target = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q_target);
            RGoal = T_target(1:3, 1:3, kin.n+2);
            tGoal = T_target(1:3, 4, kin.n+2);

            invGeoConfig.robot_model = 2;
            invGeoConfig.inv_geo_type = 0; % Analytic IK
            invGeoConfig.TI_0 = TI_0;
            invGeoConfig.inv_geo_trn = 0;
            invGeoConfig.kp_inv = 0.5;
            invGeoConfig.kr_inv = 0.5;

            q0 = zeros(kin.n, 1);
            qDes = robotics.engines.KinematicsEngine.inverseKinematics(robot, invGeoConfig, kin, RGoal, tGoal, q0);

            % Evaluate forward kinematics at solved qDes
            T_solved = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, qDes);
            t_solved = T_solved(1:3, 4, kin.n+2);
            R_solved = T_solved(1:3, 1:3, kin.n+2);

            % Position error tolerance (< 5 mm for geometric analytical solver)
            testCase.verifyEqual(t_solved, tGoal, 'AbsTol', 5e-3);
            % Rotation matrix Frobenius norm error
            testCase.verifyEqual(norm(R_solved - RGoal, 'fro'), 0.0, 'AbsTol', 5e-3);
        end

        function testForwardKinematicsHelper(testCase)
            robot = robotics.models.RobotFactory.create(1); % Franka Emika
            kin = robot.getKinematicParameters(0);
            q = deg2rad([10; -20; 30; -40; 50; -60; 70]);
            TI_0 = eye(4);

            T_ee = robotics.engines.KinematicsEngine.forwardKinematics(TI_0, kin, q);
            testCase.verifySize(T_ee, [4, 4]);

            TI_i = robotics.engines.KinematicsEngine.getTransMatrix(TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, q);
            testCase.verifyEqual(T_ee, TI_i(:, :, end), 'AbsTol', 1e-12);
        end

        function testManipulabilityAndLimitMargin(testCase)
            robot = robotics.models.RobotFactory.create(1); % Franka Emika
            kin = robot.getKinematicParameters(0);
            TI_0 = eye(4);

            % Nominal reachable pose
            q_nominal = deg2rad([0; -30; 0; -90; 0; 60; 0]);
            [w_nom, status_nom, ~] = robotics.engines.KinematicsEngine.computeManipulability(TI_0, kin, q_nominal);
            testCase.verifyGreaterThan(w_nom, 0.015);
            testCase.verifyTrue(strcmp(status_nom, 'NOMINAL') || strcmp(status_nom, 'CAUTION'));

            % Joint limit margin
            [marginDeg, closestJ] = robotics.engines.KinematicsEngine.computeLimitMargin(kin, q_nominal);
            testCase.verifyGreaterThan(marginDeg, 0);
            testCase.verifyTrue(closestJ >= 1 && closestJ <= kin.n);
        end

        function testManipulabilityEllipsoidSVD(testCase)
            robot = robotics.models.RobotFactory.create(1); % Franka Emika
            kin = robot.getKinematicParameters(0);
            TI_0 = eye(4);
            q = deg2rad([15; -25; 35; -45; 55; -65; 75]);

            [U_v, sigma_v, U_w, sigma_w] = robotics.engines.KinematicsEngine.computeManipulabilityEllipsoid(TI_0, kin, q);

            % Principal directions matrix U_v must be 3x3 and orthogonal (U'*U = I)
            testCase.verifyEqual(size(U_v), [3, 3]);
            testCase.verifyEqual(U_v' * U_v, eye(3), 'AbsTol', 1e-10);

            % Singular values must be 3x1 and non-negative
            testCase.verifyEqual(length(sigma_v), 3);
            testCase.verifyTrue(all(sigma_v >= 0));
            testCase.verifyTrue(sigma_v(1) >= sigma_v(2) && sigma_v(2) >= sigma_v(3), 'Singular values must be sorted in descending order');

            % Verify consistency with Jacobian SVD
            J = robotics.engines.KinematicsEngine.getGeometricJacobian(TI_0, kin, q);
            [~, S_exp, ~] = svd(J(1:3, :));
            testCase.verifyEqual(sigma_v, diag(S_exp(1:3, 1:3)), 'AbsTol', 1e-10);

            % Rotational ellipsoid properties
            testCase.verifyEqual(size(U_w), [3, 3]);
            testCase.verifyEqual(U_w' * U_w, eye(3), 'AbsTol', 1e-10);
            testCase.verifyTrue(all(sigma_w >= 0));
        end

        function testEllipsoidSurfaceData(testCase)
            p_ee = [0.3; 0.2; 0.5];
            U = eye(3);
            sigma = [0.8; 0.5; 0.2];
            scale = 0.15;
            n_pts = 16;

            [X, Y, Z, axesLines] = robotics.engines.KinematicsEngine.getEllipsoidSurfaceData(p_ee, U, sigma, scale, n_pts);

            testCase.verifyEqual(size(X), [n_pts + 1, n_pts + 1]);
            testCase.verifyEqual(size(Y), [n_pts + 1, n_pts + 1]);
            testCase.verifyEqual(size(Z), [n_pts + 1, n_pts + 1]);

            % Principal axes lines count and dimensions
            testCase.verifyEqual(length(axesLines), 3);
            for i = 1:3
                testCase.verifyEqual(size(axesLines{i}), [3, 2]);
            end
        end

    end
end
