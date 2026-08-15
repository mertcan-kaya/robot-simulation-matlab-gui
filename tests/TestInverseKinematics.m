classdef TestInverseKinematics < matlab.unittest.TestCase
    % Tests for the Inverse Kinematics module
    
    methods(Test)
        function testNumericIK(testCase)
            robot = robotics.models.RobotFactory.create(1); % Franka
            kin = robot.getKinematicParameters(1);
            
            invGeoConfig.robot_model = 1;
            invGeoConfig.inv_geo_type = 1; % Numeric IK
            invGeoConfig.TI_0 = eye(4);
            invGeoConfig.inv_geo_trn = 0; % Jacobian transpose
            invGeoConfig.kp_inv = 0.5;
            invGeoConfig.kr_inv = 0.5;
            
            RGoal = eye(3);
            tGoal = [0.3; 0.1; 0.5];
            q0 = zeros(kin.n, 1);
            
            robot = robotics.models.RobotFactory.create(invGeoConfig.robot_model);
            qDes = robotics.engines.KinematicsEngine.inverseKinematics(robot, invGeoConfig, kin, RGoal, tGoal, q0);
            
            testCase.verifySize(qDes, [kin.n, 1]);
        end
        
        function testAnalyticIK_UR3(testCase)
            robot = robotics.models.RobotFactory.create(2); % UR3
            kin = robot.getKinematicParameters(1);
            
            invGeoConfig.robot_model = 2;
            invGeoConfig.inv_geo_type = 0; % Analytic IK
            invGeoConfig.TI_0 = eye(4);
            invGeoConfig.inv_geo_trn = 0; % Jacobian transpose
            invGeoConfig.kp_inv = 0.5;
            invGeoConfig.kr_inv = 0.5;
            
            % Define a valid target configuration
            qTarget = [0.1; -0.2; 0.3; -0.4; 0.5; 0.6];
            
            % Forward kinematics to find the target position
            T = robotics.engines.KinematicsEngine.getTransMatrix(invGeoConfig.TI_0, kin.a_j, kin.alpha_j, kin.d_j, kin.theta_O_j, kin.j_type, qTarget);
            RGoal = T(1:3, 1:3, kin.n+2);
            tGoal = T(1:3, 4, kin.n+2);
            
            q0 = zeros(kin.n, 1);
            
            robot = robotics.models.RobotFactory.create(invGeoConfig.robot_model);
            qDes = robotics.engines.KinematicsEngine.inverseKinematics(robot, invGeoConfig, kin, RGoal, tGoal, q0);
            
            % Verify the output size
            testCase.verifySize(qDes, [kin.n, 1]);
            
            % Verify the result is within joint limits
            testCase.verifyTrue(all(qDes >= kin.q_posLim(:,1)));
            testCase.verifyTrue(all(qDes <= kin.q_posLim(:,2)));
        end
    end
end

