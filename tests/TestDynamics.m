classdef TestDynamics < matlab.unittest.TestCase
    % TESTDYNAMICS Unit tests for forward and inverse dynamics solvers in +robotics/+engines.

    methods (Test)

        function testGravityCompensationEquilibrium(testCase)
            % Test that applying tau = G(q) cancels gravity and results in zero acceleration (q_acc == 0)
            robot = robotics.models.RobotFactory.create(1); % Franka Emika
            kin = robot.getKinematicParameters(0);
            dyn = robot.getInertialParameters();
            
            % Disable friction and springs for pure rigid-body dynamics test
            dyn.spring_on = 0;
            dyn.friction_on = 0;

            q_test = [0.2; -0.3; 0.4; -0.5; 0.6; -0.7; 0.8];
            q_vel_zero = zeros(kin.n, 1);
            g = [0; 0; -9.81];

            % Compute gravity compensation torque
            tau_g = robotics.engines.DynamicsEngine.getTauG(kin, q_test, g, dyn.m_j, dyn.dj_j);
            testCase.verifySize(tau_g, [kin.n, 1]);

            % Compute forward dynamics with tau = tau_g
            q_acc = robotics.engines.DynamicsEngine.forwardDynamics(robot, kin, dyn, tau_g, q_test, q_vel_zero);
            testCase.verifySize(q_acc, [kin.n, 1]);

            % Accelerations should be identically zero at static equilibrium under gravity cancellation
            testCase.verifyEqual(q_acc, zeros(kin.n, 1), 'AbsTol', 1e-4);
        end

        function testSpringTorque(testCase)
            % Test spring torque computation for robot models
            robot_ur3 = robotics.models.RobotFactory.create(2);
            q_pos = [0.5; -0.5; 0.5; -0.5; 0.5; -0.5];
            
            tau_spr_ur3 = robot_ur3.getSpringTorque(q_pos);
            testCase.verifySize(tau_spr_ur3, [6, 1]);

            % Staubli RX160 has gravity compensation springs
            robot_staubli = robotics.models.RobotFactory.create(4);
            tau_spr_staubli = robot_staubli.getSpringTorque(q_pos);
            testCase.verifySize(tau_spr_staubli, [6, 1]);
        end

        function testFrictionTorque(testCase)
            % Test friction torque computation for non-zero joint velocities
            robot = robotics.models.RobotFactory.create(1);
            q_vel = [1.0; -1.5; 0.5; -0.2; 0.8; -0.4; 0.1];

            tau_frc = robot.getFrictionTorque(q_vel);
            testCase.verifySize(tau_frc, [7, 1]);

            % Opposing friction sign check: sign(tau_frc) should match sign(q_vel)
            for j = 1:7
                if abs(q_vel(j)) > 0.05
                    testCase.verifyEqual(sign(tau_frc(j)), sign(q_vel(j)));
                end
            end
        end

        function testInverseDynamicsMNEA(testCase)
            % Test Modified Newton-Euler inverse dynamics
            robot = robotics.models.RobotFactory.create(2); % UR3
            kin = robot.getKinematicParameters(0);
            dyn = robot.getInertialParameters();

            q = zeros(kin.n, 1);
            qd = zeros(kin.n, 1);
            qRd = zeros(kin.n, 1);
            qRdd = ones(kin.n, 1);
            g = [0; 0; -9.81];

            tau_j = robotics.engines.DynamicsEngine.inverseDynamicsMNEA(kin, q, qd, qRd, qRdd, g, dyn.pj_j);
            testCase.verifySize(tau_j, [kin.n, 1]);
            testCase.verifyFalse(any(isnan(tau_j)));
            testCase.verifyFalse(any(isinf(tau_j)));
        end

        function testInverseDynamicsANEA(testCase)
            % Test Adaptive Newton-Euler inverse dynamics
            robot = robotics.models.RobotFactory.create(2); % UR3
            kin = robot.getKinematicParameters(0);
            dyn = robot.getInertialParameters();

            ctr.tcyc = 0.001;
            q = zeros(kin.n, 1);
            qd = zeros(kin.n, 1);
            qRd = zeros(kin.n, 1);
            qRdd = ones(kin.n, 1);
            g = [0; 0; -9.81];

            p_init = dyn.pj_j(:);
            Pdiag = ones(length(p_init), 1) * 0.01;

            [tau_fj, phat_next] = robotics.engines.DynamicsEngine.inverseDynamicsANEA(ctr, kin, q, qd, qRd, qRdd, g, p_init, Pdiag);
            testCase.verifySize(tau_fj, [kin.n, 1]);
            testCase.verifySize(phat_next, size(p_init));
            testCase.verifyFalse(any(isnan(phat_next)));
        end

    end
end
