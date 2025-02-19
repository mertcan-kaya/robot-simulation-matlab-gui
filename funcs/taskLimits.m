function [t_posLim,t_posSafeLim,t_velLim,t_velSafeLim,t_accLim] = taskLimits(robot_model,DH)

    if robot_model == 5 % Stäubli RX160L
        % task position limits (m): 
        x_max = DH(2,1)+DH(3,1)+DH(4,3)+DH(7,3); % [m]
        x_max_ = x_max - 0.1;  % give a safety margin of 0.1 m
        x_min = -x_max; % [m]
        x_min_ = x_min + 0.1;  % give a safety margin of 0.1 m
        y_max = DH(2,1)+DH(3,1)+DH(4,3)+DH(7,3); % [m]
        y_max_ = y_max - 0.1;  % give a safety margin of 0.1 m
        y_min = -y_max; % [m]
        y_min_ = y_min + 0.1;  % give a safety margin of 0.1 m
        z_max = DH(1,3)+DH(3,1)+DH(4,3)+DH(7,3); % [m]
        z_max_ = z_max - 0.1;  % give a safety margin of 0.1 m
        z_min = DH(1,3)-DH(3,1)-DH(4,3)-DH(7,3); % [m]
        z_min_ = z_min + 0.1;  % give a safety margin of 0.1 m
        % task velocity limits (m/s):
        dt_max = 1;
        dt_max_ = dt_max - 0.1;
        dt_min = -dt_max;
        dt_min_ = -dt_max_;
        % task acceleration limits (m/s^2): (q1..q6)
        ddt_max = 0.1;
        ddt_min = -ddt_max; 
    elseif robot_model == 4 % Stäubli RX160
        % task position limits (m): 
        x_max = DH(2,1)+DH(3,1)+DH(4,3)+DH(7,3); % [m]
        x_max_ = x_max - 0.1;  % give a safety margin of 0.1 m
        x_min = -x_max; % [m]
        x_min_ = x_min + 0.1;  % give a safety margin of 0.1 m
        y_max = DH(2,1)+DH(3,1)+DH(4,3)+DH(7,3); % [m]
        y_max_ = y_max - 0.1;  % give a safety margin of 0.1 m
        y_min = -y_max; % [m]
        y_min_ = y_min + 0.1;  % give a safety margin of 0.1 m
        z_max = DH(1,3)+DH(3,1)+DH(4,3)+DH(7,3); % [m]
        z_max_ = z_max - 0.1;  % give a safety margin of 0.1 m
        z_min = DH(1,3)-DH(3,1)-DH(4,3)-DH(7,3); % [m]
        z_min_ = z_min + 0.1;  % give a safety margin of 0.1 m
        % task velocity limits (m/s):
        dt_max = 1;
        dt_max_ = dt_max - 0.1;
        dt_min = -dt_max;
        dt_min_ = -dt_max_;
        % task acceleration limits (m/s^2): (q1..q6)
        ddt_max = 0.1;
        ddt_min = -ddt_max; 
    elseif robot_model == 3 % Unitree Z1
        % task position limits (m): 
        x_max = DH(3,1)+DH(4,1)+DH(5,1)+DH(7,3); % [m]
        x_max_ = x_max - 0.1;  % give a safety margin of 0.1 m
        x_min = -x_max; % [m]
        x_min_ = x_min + 0.1;  % give a safety margin of 0.1 m
        y_max = DH(3,1)+DH(4,1)+DH(5,1)+DH(7,3); % [m]
        y_max_ = y_max - 0.1;  % give a safety margin of 0.1 m
        y_min = -y_max; % [m]
        y_min_ = y_min + 0.1;  % give a safety margin of 0.1 m
        z_max = DH(1,3)+DH(3,1)+DH(4,1)+DH(5,1)+DH(7,3); % [m]
        z_max_ = z_max - 0.1;  % give a safety margin of 0.1 m
        z_min = DH(1,3)-DH(3,1)-DH(4,1)-DH(5,1)-DH(7,3); % [m]
        z_min_ = z_min + 0.1;  % give a safety margin of 0.1 m
        % task velocity limits (m/s):
        dt_max = 1;
        dt_max_ = dt_max - 0.1;
        dt_min = -dt_max;
        dt_min_ = -dt_max_;
        % task acceleration limits (m/s^2): (q1..q6)
        ddt_max = 0.1;
        ddt_min = -ddt_max; 
    elseif robot_model == 2 % Universal Robots U3
        % task position limits (m): 
        x_max = DH(3,1)+DH(4,1)+DH(7,3); % [m]
        x_max_ = x_max - 0.1;  % give a safety margin of 0.1 m
        x_min = -x_max; % [m]
        x_min_ = x_min + 0.1;  % give a safety margin of 0.1 m
        y_max = DH(3,1)+DH(4,1)+DH(7,3); % [m]
        y_max_ = y_max - 0.1;  % give a safety margin of 0.1 m
        y_min = -y_max; % [m]
        y_min_ = y_min + 0.1;  % give a safety margin of 0.1 m
        z_max = DH(1,3)+DH(3,1)+DH(4,1)+DH(7,3); % [m]
        z_max_ = z_max - 0.1;  % give a safety margin of 0.1 m
        z_min = DH(1,3)-DH(3,1)-DH(4,1)-DH(7,3); % [m]
        z_min_ = z_min + 0.1;  % give a safety margin of 0.1 m
        % task velocity limits (m/s):
        dt_max = 1;
        dt_max_ = dt_max - 0.1;
        dt_min = -dt_max;
        dt_min_ = -dt_max_;
        % task acceleration limits (m/s^2): (q1..q6)
        ddt_max = 0.1;
        ddt_min = -ddt_max; 
    elseif robot_model == 1 % Franka Emika Robot
        % task position limits (m): 
        x_max = DH(3,3)+DH(5,3)+DH(8,3); % [m]
        x_max_ = x_max - 0.1;  % give a safety margin of 0.1 m
        x_min = -x_max; % [m]
        x_min_ = x_min + 0.1;  % give a safety margin of 0.1 m
        y_max = DH(3,3)+DH(5,3)+DH(8,3); % [m]
        y_max_ = y_max - 0.1;  % give a safety margin of 0.1 m
        y_min = -y_max; % [m]
        y_min_ = y_min + 0.1;  % give a safety margin of 0.1 m
        z_max = DH(1,3)+DH(3,3)+DH(5,3)+DH(8,3); % [m]
        z_max_ = z_max - 0.1;  % give a safety margin of 0.1 m
        z_min = DH(1,3)-DH(3,3)-DH(5,3)-DH(8,3); % [m]
        z_min_ = z_min + 0.1;  % give a safety margin of 0.1 m
        % task velocity limits (m/s):
        dt_max = 1;
        dt_max_ = dt_max - 0.1;
        dt_min = -dt_max;
        dt_min_ = -dt_max_;
        % task acceleration limits (m/s^2): (q1..q6)
        ddt_max = 0.1;
        ddt_min = -ddt_max; 
    else % Custom Robot
        % task position limits (m): 
        x_max = sum(DH(:,1))+sum(DH(:,3)); % [m]
        x_max_ = x_max - 0.1;  % give a safety margin of 0.1 m
        x_min = -x_max; % [m]
        x_min_ = x_min + 0.1;  % give a safety margin of 0.1 m
        y_max = sum(DH(:,1))+sum(DH(:,3)); % [m]
        y_max_ = y_max - 0.1;  % give a safety margin of 0.1 m
        y_min = -y_max; % [m]
        y_min_ = y_min + 0.1;  % give a safety margin of 0.1 m
        z_max = sum(DH(:,1))+sum(DH(:,3)); % [m]
        z_max_ = z_max - 0.1;  % give a safety margin of 0.1 m
        z_min = -sum(DH(:,1))-sum(DH(:,3)); % [m]
        z_min_ = z_min + 0.1;  % give a safety margin of 0.1 m
        % task velocity limits (m/s):
        dt_max = 1;
        dt_max_ = dt_max - 0.1;
        dt_min = -dt_max;
        dt_min_ = -dt_max_;
        % task acceleration limits (m/s^2): (q1..q6)
        ddt_max = 0.1;
        ddt_min = -ddt_max; 
    end

    t_posLim = [x_min,x_max;y_min,y_max;z_min,z_max];
    t_posSafeLim = [x_min_,x_max_;y_min_,y_max_;z_min_,z_max_];
    t_velLim = [dt_min,dt_max];
    t_velSafeLim = [dt_min_,dt_max_];
    t_accLim = [ddt_min,ddt_max];

end