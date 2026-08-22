classdef TestControlEngine < matlab.unittest.TestCase
    % TESTCONTROLENGINE Unit tests for ControlEngine and KinematicsEngine Jacobian/orientation methods.

    properties
        Robot
        Kin
        Dyn
        TI_0
    end

    methods (TestMethodSetup)
        function setupRobot(testCase)
            testCase.Robot = robotics.models.RobotFactory.create(1); % Franka Emika (7 DoF)
            testCase.Kin = testCase.Robot.getKinematicParameters(0);
            testCase.Kin.g0 = -[0; 0; 9.81];
            testCase.Dyn = testCase.Robot.getInertialParameters();
            testCase.TI_0 = eye(4);
        end
    end

    methods (Test)
        function testGeometricJacobian(testCase)
            q = [0.1; -0.2; 0.3; -0.4; 0.5; 0.6; -0.7];
            J = robotics.engines.KinematicsEngine.getGeometricJacobian(testCase.TI_0, testCase.Kin, q);

            testCase.verifyEqual(size(J), [6, 7]);

            % Verify linear Jacobian J_v via numerical finite differences
            eps_val = 1e-6;
            TI_nom = robotics.engines.KinematicsEngine.getTransMatrix(...
                testCase.TI_0, testCase.Kin.a_j, testCase.Kin.alpha_j, testCase.Kin.d_j, ...
                testCase.Kin.theta_O_j, testCase.Kin.j_type, q);
            p_nom = TI_nom(1:3, 4, end);

            for i = 1:7
                q_pert = q;
                q_pert(i) = q_pert(i) + eps_val;
                TI_pert = robotics.engines.KinematicsEngine.getTransMatrix(...
                    testCase.TI_0, testCase.Kin.a_j, testCase.Kin.alpha_j, testCase.Kin.d_j, ...
                    testCase.Kin.theta_O_j, testCase.Kin.j_type, q_pert);
                p_pert = TI_pert(1:3, 4, end);
                Jv_num = (p_pert - p_nom) / eps_val;

                testCase.verifyEqual(J(1:3, i), Jv_num, 'AbsTol', 1e-4);
            end
        end

        function testOrientationError(testCase)
            R1 = eye(3);
            R2 = eye(3);
            e_o_zero = robotics.engines.KinematicsEngine.computeOrientationError(R1, R2);
            testCase.verifyEqual(e_o_zero, zeros(3, 1), 'AbsTol', 1e-10);

            % Small rotation around X by theta
            theta = 0.05;
            Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
            e_o = robotics.engines.KinematicsEngine.computeOrientationError(Rx, eye(3));
            testCase.verifyEqual(e_o(1), sin(theta), 'AbsTol', 1e-4);
            testCase.verifyEqual(e_o(2:3), [0; 0], 'AbsTol', 1e-6);
        end

        function testJointSpacePIDControl(testCase)
            ctr = testCase.Robot.getDefaultControlParams(0);
            ctr.space = 0; % Joint Space
            ctr.algo = 0;  % PID
            ctr.comp_grv = 0;

            des.q_pos = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6; 0.7];
            des.q_vel = zeros(7, 1);
            des.q_acc = zeros(7, 1);

            fbk.q_pos = zeros(7, 1);
            fbk.q_vel = zeros(7, 1);

            tau = robotics.engines.ControlEngine.computeTorque(...
                ctr, testCase.Kin, testCase.Dyn, des, fbk, testCase.TI_0);

            testCase.verifyEqual(length(tau), 7);
            testCase.verifyTrue(all(tau > 0));
        end

        function testTaskSpacePIDControl(testCase)
            ctr = testCase.Robot.getDefaultControlParams(0);
            ctr.space = 1; % Task Space
            ctr.algo = 0;  % PID
            ctr.comp_grv = 0;

            des.q_pos = [0.1; 0.2; 0.3; -0.4; 0.5; 0.6; 0.7];
            des.q_vel = zeros(7, 1);
            des.q_acc = zeros(7, 1);

            fbk.q_pos = zeros(7, 1);
            fbk.q_vel = zeros(7, 1);

            tau = robotics.engines.ControlEngine.computeTorque(...
                ctr, testCase.Kin, testCase.Dyn, des, fbk, testCase.TI_0);

            testCase.verifyEqual(length(tau), 7);
            testCase.verifyFalse(any(isnan(tau)));
            testCase.verifyFalse(any(isinf(tau)));
        end

        function testTaskSpaceIDCControl(testCase)
            ctr = testCase.Robot.getDefaultControlParams(1);
            ctr.space = 1; % Task Space
            ctr.algo = 1;  % IDC / CTC
            ctr.comp_grv = 0;

            des.q_pos = [0.1; 0.2; 0.3; -0.4; 0.5; 0.6; 0.7];
            des.q_vel = zeros(7, 1);
            des.q_acc = zeros(7, 1);

            fbk.q_pos = zeros(7, 1);
            fbk.q_vel = zeros(7, 1);

            tau = robotics.engines.ControlEngine.computeTorque(...
                ctr, testCase.Kin, testCase.Dyn, des, fbk, testCase.TI_0);

            testCase.verifyEqual(length(tau), 7);
            testCase.verifyFalse(any(isnan(tau)));
            testCase.verifyFalse(any(isinf(tau)));
        end

        function testGainTuningEngineAcrossAllRobots(testCase)
            for robotId = 1:5
                robot = robotics.models.RobotFactory.create(robotId);
                kin = robot.getKinematicParameters(0);
                
                % Compute optimal gains
                ctr = robotics.engines.GainTuningEngine.computeOptimalGains(robot);
                
                % Verify dimensions and non-negativity
                testCase.verifyEqual(length(ctr.Kp_jnt_pid), kin.n);
                testCase.verifyEqual(length(ctr.Kd_jnt_pid), kin.n);
                testCase.verifyTrue(all(ctr.Kp_jnt_pid > 0));
                testCase.verifyTrue(all(ctr.Kd_jnt_pid > 0));
                
                testCase.verifyEqual(length(ctr.Kp_jnt_idc), kin.n);
                testCase.verifyEqual(length(ctr.Kd_jnt_idc), kin.n);
                testCase.verifyTrue(all(ctr.Kp_jnt_idc > 0));
                testCase.verifyTrue(all(ctr.Kd_jnt_idc > 0));
                
                testCase.verifyEqual(length(ctr.Kp_tsk_pid), 6);
                testCase.verifyEqual(length(ctr.Kd_tsk_pid), 6);
                testCase.verifyTrue(all(ctr.Kp_tsk_pid > 0));
                testCase.verifyTrue(all(ctr.Kd_tsk_pid > 0));
                
                testCase.verifyEqual(length(ctr.Kp_tsk_idc), 6);
                testCase.verifyEqual(length(ctr.Kd_tsk_idc), 6);
                testCase.verifyTrue(all(ctr.Kp_tsk_idc > 0));
                testCase.verifyTrue(all(ctr.Kd_tsk_idc > 0));
            end
        end
    end
end
