function [q_posLim,q_posSafeLim,q_velLim,q_velSafeLim,q_accLim] = jointLimits(robot_model)
    % Wrapper that delegates to the Object-Oriented Robot Models
    robot = RobotFactory.create(robot_model);
    [q_posLim, q_posSafeLim, q_velLim, q_velSafeLim, q_accLim] = robot.getJointLimits();
end