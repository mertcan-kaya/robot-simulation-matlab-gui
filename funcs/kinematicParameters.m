function kin = kinematicParameters(robot_model, ee_att)
    % Wrapper that delegates to the Object-Oriented Robot Models
    robot = RobotFactory.create(robot_model);
    kin = robot.getKinematicParameters(ee_att);
end