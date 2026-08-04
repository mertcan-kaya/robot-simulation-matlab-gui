function qDes = invGeoCore(invGeoConfig, kin, RGoal, tGoal, q0)
    % Instantiate the correct robot object using the Factory
    robot = RobotFactory.create(invGeoConfig.robot_model);
    
    % Delegate inverse kinematics to the robot object
    [qDes, ~] = robot.computeInverseKinematics(invGeoConfig, kin, RGoal, tGoal, q0);
end
