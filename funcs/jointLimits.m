function [q_posLim,q_posSafeLim,q_velLim,q_velSafeLim,q_accLim] = jointLimits(robot_model)

    if robot_model == 5
        % joint limits (rad): (q1..q6)
        q_min = [-2.967060 -2.4 -2.62 -4.71 -1.83 -4.71]'; % [rad]
        q_min_ = q_min + ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        q_max = [2.967060 2.4 2.62 4.71 2.09 4.71]'; % [rad]
        q_max_ = q_max - ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        % joint velocity limits (rad/s): (q1..q6)
        dq_max = [2.88 2.62 3.32 5.15 4.54 7.68]';
        dq_max_ = dq_max - ones(6,1)*deg2rad(10);
        dq_min = -dq_max;
        dq_min_ = -dq_max_;
        % joint acceleration limits (rad/s^2): (q1..q6)
        ddq_max = [25 25 25 25 25 25]';
        ddq_min = -ddq_max; 
    elseif robot_model == 4
        % joint limits (rad): (q1..q6)
        q_min = [-2.967060 -2.4 -2.62 -4.71 -1.83 -4.71]'; % [rad]
        q_min_ = q_min + ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        q_max = [2.967060 2.4 2.62 4.71 2.09 4.71]'; % [rad]
        q_max_ = q_max - ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        % joint velocity limits (rad/s): (q1..q6)
        dq_max = [2.88 2.62 3.32 5.15 4.54 7.68]';
        dq_max_ = dq_max - ones(6,1)*deg2rad(10);
        dq_min = -dq_max;
        dq_min_ = -dq_max_;
        % joint acceleration limits (rad/s^2): (q1..q6)
        ddq_max = [25 25 25 25 25 25]';
        ddq_min = -ddq_max; 
    elseif robot_model == 3
        % joint limits (deg): (q1..q6)
        %       [-150 0 -165 -80 -85 -160] [deg]
        q_min = [-2.6180 0 -2.8798 -1.3963 -1.4835 -2.7925]'; % [rad]
        q_min_ = q_min + ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        %       [150 180 0 80 85 160] [deg]
        q_max = [2.6180 3.1416 0 1.3963 1.4835 2.7925]'; % [rad]
        q_max_ = q_max - ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        % joint velocity limits (rad/s): (q1..q6)
        dq_max = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]';
        dq_max_ = dq_max - ones(6,1)*deg2rad(10);
        dq_min = -dq_max;
        dq_min_ = -dq_max_;
        % joint acceleration limits (rad/s^2): (q1..q6)
        ddq_max = [10 10 10 10 10 10]';
        ddq_min = -ddq_max; 
    elseif robot_model == 2
        % joint limits (deg): (q1..q6)
        q_min = [-pi   -pi   -pi   -pi   -pi   -pi]'; % [rad]
        q_min_ = q_min + ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        q_max = [ pi    pi    pi    pi    pi    pi]'; % [rad]
        q_max_ = q_max - ones(6,1)*deg2rad(5);  % give a safety margin of 5 deg
        % joint velocity limits (rad/s): (q1..q6)
        dq_max = [pi/2 pi/2 pi/2 pi pi pi]';
        dq_max_ = dq_max - ones(6,1)*deg2rad(10);
        dq_min = -dq_max;
        dq_min_ = -dq_max_;
        % joint acceleration limits (rad/s^2): (q1..q6)
        ddq_max = [10 10 10 20 20 20]';
        ddq_min = -ddq_max; 
    else
        % joint limits (deg): (q1..q7)
        %       [-166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027 -166.0031] [deg]
        q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973]'; % [rad]
        q_min_ = q_min + ones(7,1)*deg2rad(5);  % give a safety margin of 5 deg
        %       [166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024 166.0031] [deg]
        q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973]'; % [rad]
        q_max_ = q_max - ones(7,1)*deg2rad(5);  % give a safety margin of 5 deg
        % joint velocity limits (rad/s): (q1..q7)
        dq_max = [2.1750 2.1750 2.1750 2.1750 2.6100 2.6100 2.6100]';
        dq_max_ = dq_max - ones(7,1)*deg2rad(10);
        dq_min = -dq_max;
        dq_min_ = -dq_max_;
        % joint acceleration limits (rad/s^2): (q1..q7)
        ddq_max = [15 7.5 10 12.5 15 20 20]';
        ddq_min = -ddq_max; 
    end

    q_posLim = [q_min,q_max];
    q_posSafeLim = [q_min_,q_max_];
    q_velLim = [dq_min,dq_max];
    q_velSafeLim = [dq_min_,dq_max_];
    q_accLim = [ddq_min,ddq_max];

end