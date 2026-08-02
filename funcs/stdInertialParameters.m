function dyn = stdInertialParameters(robot_model)
    % Wrapper that delegates to the Object-Oriented Robot Models
    robot = RobotFactory.create(robot_model);
    dyn = robot.getInertialParameters();
end