function [t_posLim,t_posSafeLim,t_velLim,t_velSafeLim,t_accLim] = taskLimits(robot_model, DH)
    % Wrapper that delegates to the Object-Oriented Robot Models
    robot = RobotFactory.create(robot_model);
    [t_posLim, t_posSafeLim, t_velLim, t_velSafeLim, t_accLim] = robot.getTaskLimits(DH);
end