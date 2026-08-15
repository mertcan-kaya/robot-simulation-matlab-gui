classdef TestRobotModels < matlab.unittest.TestCase
    % Tests for the Object-Oriented Robot Models
    
    properties
        ExpectedClasses = {
            'robotics.models.FrankaEmika', ...
            'robotics.models.UR3', ...
            'robotics.models.UnitreeZ1', ...
            'robotics.models.StaubliRX160', ...
            'robotics.models.StaubliRX160L', ...
            'robotics.models.CustomRobot'
        };
    end
    
    methods(Test)
        
        function testRobotFactory(testCase)
            % Verify that robotics.models.RobotFactory returns the correct subclasses
            for id = 1:6
                robot = robotics.models.RobotFactory.create(id);
                testCase.verifyClass(robot, testCase.ExpectedClasses{id});
            end
        end
        
        function testJointLimitsAndKinematics(testCase)
            % Verify that getJointLimits and getKinematicParameters return correctly shaped arrays
            for id = 1:6
                robot = robotics.models.RobotFactory.create(id);
                kin = robot.getKinematicParameters(1); % Test with ee_att = 1
                
                % Verify kin properties exist
                testCase.verifyTrue(isfield(kin, 'a_j'));
                testCase.verifyTrue(isfield(kin, 'n'));
                
                n = kin.n; % number of joints
                
                [q_posLim, q_posSafeLim, q_velLim, q_velSafeLim, q_accLim] = robot.getJointLimits();
                
                % Verify sizes
                testCase.verifySize(q_posLim, [n, 2]);
                testCase.verifySize(q_posSafeLim, [n, 2]);
                testCase.verifySize(q_velLim, [n, 2]);
                testCase.verifySize(q_velSafeLim, [n, 2]);
                testCase.verifySize(q_accLim, [n, 2]);
            end
        end
        
        function testInertialParameters(testCase)
            % Verify that getInertialParameters returns correctly shaped structs
            for id = 1:6
                robot = robotics.models.RobotFactory.create(id);
                dyn = robot.getInertialParameters();
                
                testCase.verifyTrue(isfield(dyn, 'm_j'));
                testCase.verifyTrue(isfield(dyn, 'Ij_cj'));
            end
        end
        
    end
end




