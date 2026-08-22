classdef SimulationEngine < handle
    % SIMULATIONENGINE Executes time-stepping simulation loops for dynamic and kinematic modes.
    %   Orchestrates forward dynamics integration (Euler/RK), control torque dispatch,
    %   trajectory evaluation, joint limit enforcement, real-time wall-clock pacing,
    %   and hardware-accelerated 3D graphics rendering.
    
    properties
        model
        robot
        renderer
        onUpdateLabels
    end
    
    methods
        function obj = SimulationEngine(model, robot, renderer, onUpdateLabels)
            % SIMULATIONENGINE Constructs an instance of the simulation engine.
            %
            %   Inputs:
            %       model          - SimulationModel instance (simulation state and params)
            %       robot          - RobotModel instance (physics and analytical models)
            %       renderer       - RobotRenderer instance (3D graphics pipeline) (optional)
            %       onUpdateLabels - function_handle: UI callback for stats (optional)
            
            if nargin < 3, renderer = []; end
            if nargin < 4, onUpdateLabels = []; end
            
            obj.model = model;
            obj.robot = robot;
            obj.renderer = renderer;
            obj.onUpdateLabels = onUpdateLabels;
        end

        function runDynamics(obj)
            % RUNDYNAMICS Alias for runSimulation (dynamic forward simulation loop).
            obj.runSimulation();
        end
        
        function runSimulation(obj)
            % RUNSIMULATION Executes the full forward dynamic numerical simulation loop.
            %   Integrates forward dynamics (q_acc -> q_vel -> q_pos), computes feedback
            %   control torques, enforces joint limits, and synchronizes rendering to real-time.
            
            obj.model.running_flag = 1;
            
            d = round(obj.model.ctr.tcyc/obj.model.tstp);
            obj.model.ctr.errsum = zeros(obj.model.kin.n,1);
            
            obj.model.act.q_pos = obj.model.ini.q_pos;
            obj.model.act.q_vel = zeros(obj.model.kin.n,1);
            obj.model.act.q_acc = zeros(obj.model.kin.n,1);
            
            trjConfig.tstp = obj.model.tstp;
            trjConfig.tfin_trj = obj.model.tfin_trj;
            trjConfig.trj_profile = obj.model.trj_profile;
            
            speed_scale = obj.model.time_const;
            if isempty(speed_scale) || speed_scale <= 0
                speed_scale = 1.0;
            end
            
            target_fps = 100; % High-framerate rendering cap
            total_steps = round(obj.model.tfin / obj.model.tstp);
            
            if isinf(speed_scale)
                steps_per_frame = total_steps;
            else
                dt_frame_real = 1.0 / target_fps;
                dt_frame_sim = dt_frame_real * speed_scale;
                steps_per_frame = max(1, round(dt_frame_sim / obj.model.tstp));
            end
            
            last_render_wall_time = -inf;
            last_label_wall_time = -inf;
            fps_smoothed = target_fps;
            actual_rtf = speed_scale;
            
            dyn_ws = robotics.engines.DynamicsEngine.createDynamicsWorkspace(obj.model.kin.n);
            
            % Pre-allocate diagnostic log arrays
            log_decim = max(1, round(0.001 / obj.model.tstp));
            max_log_len = floor(total_steps / log_decim) + 2;
            log_time = zeros(max_log_len, 1);
            log_q_des = zeros(max_log_len, obj.model.kin.n);
            log_q_act = zeros(max_log_len, obj.model.kin.n);
            log_q_vel_des = zeros(max_log_len, obj.model.kin.n);
            log_q_vel_act = zeros(max_log_len, obj.model.kin.n);
            log_q_acc_des = zeros(max_log_len, obj.model.kin.n);
            log_q_acc_act = zeros(max_log_len, obj.model.kin.n);
            log_tau = zeros(max_log_len, obj.model.kin.n);
            log_p_des = zeros(max_log_len, 3);
            log_p_act = zeros(max_log_len, 3);
            log_idx = 0;
            
            tic;
            for k = 0:total_steps
                
                if obj.model.running_flag == 0
                    break;
                end
                
                % Low frequency control loop
                if rem(k, d) == 0
                    obj.model.fbk.q_vel = obj.model.act.q_vel;
                    obj.model.fbk.q_pos = obj.model.act.q_pos;
                    
                    if obj.model.trj_on == 0
                        % Live reference tracking mode (no trajectory interpolation)
                        obj.model.des.q_pos = obj.model.fin.q_pos;
                        obj.model.des.q_vel = zeros(obj.model.kin.n, 1);
                        obj.model.des.q_acc = zeros(obj.model.kin.n, 1);
                        obj.model.des.t_pos = obj.model.fin.t_pos;
                        obj.model.des.Re    = obj.model.fin.Re;
                        obj.model.des.x_vel = zeros(6, 1);
                        obj.model.des.x_acc = zeros(6, 1);
                    else
                        % Trajectory interpolation mode (Joint vs Task)
                        if obj.model.trj_space == 1
                            invConfig.inv_geo_type = obj.model.inv_geo_type;
                            invConfig.robot_model = obj.model.robot_model;
                            invConfig.TI_0 = obj.model.TI_0;
                            invConfig.inv_geo_trn = obj.model.inv_geo_trn;
                            invConfig.kp_inv = obj.model.kp_inv;
                            invConfig.kr_inv = obj.model.kr_inv;
                            invConfig.kp_trn = obj.model.kp_trn;
                            invConfig.kr_trn = obj.model.kr_trn;
                            
                            ini_x = [obj.model.ini.t_pos; obj.model.ini.r_pos];
                            fin_x = [obj.model.fin.t_pos; obj.model.fin.r_pos];
                            
                            [obj.model.des.q_pos, obj.model.des.q_vel, obj.model.des.q_acc, x_pos, x_vel, x_acc] = ...
                                robotics.engines.TrajectoryEngine.generateTaskSpaceTrajectory(...
                                    trjConfig, obj.robot, invConfig, obj.model.kin, ini_x, fin_x, ...
                                    obj.model.fbk.q_pos, k, obj.model.eulerSet);
                            obj.model.des.t_pos = x_pos(1:3);
                            obj.model.des.Re    = robotics.math.getRotMatfromEA(x_pos(4:6), obj.model.eulerSet);
                            obj.model.des.x_vel = x_vel;
                            obj.model.des.x_acc = x_acc;
                        else
                            [obj.model.des.q_pos, obj.model.des.q_vel, obj.model.des.q_acc] = ...
                                robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, obj.model.kin, obj.model.ini.q_pos, obj.model.fin.q_pos, k);
                            obj.model.des.t_pos = [];
                            obj.model.des.Re    = [];
                            obj.model.des.x_vel = [];
                            obj.model.des.x_acc = [];
                        end
                    end
                end
                
                tau = robotics.engines.ControlEngine.computeTorque(obj.model.ctr, obj.model.kin, obj.model.dyn, obj.model.des, obj.model.fbk, obj.model.TI_0);
                
                % High frequency physics loop (zero-allocation)
                obj.model.act.q_acc = robotics.engines.DynamicsEngine.forwardDynamics(obj.robot, obj.model.kin, obj.model.dyn, tau, obj.model.act.q_pos, obj.model.act.q_vel, dyn_ws);
                obj.model.act.q_vel = obj.model.act.q_vel + obj.model.act.q_acc*obj.model.tstp;
                obj.model.act.q_pos = obj.model.act.q_pos + obj.model.act.q_vel*obj.model.tstp;
                
                % Joint Limits
                for i = 1:obj.model.kin.n
                    if obj.model.act.q_pos(i) < obj.model.kin.q_posLim(i,1)
                        obj.model.act.q_pos(i) = obj.model.kin.q_posLim(i,1);
                        obj.model.act.q_vel(i) = 0;
                        obj.model.act.q_acc(i) = 0;
                    elseif obj.model.act.q_pos(i) > obj.model.kin.q_posLim(i,2)
                        obj.model.act.q_pos(i) = obj.model.kin.q_posLim(i,2);
                        obj.model.act.q_vel(i) = 0;
                        obj.model.act.q_acc(i) = 0;
                    end
                end
                
                % Telemetry data logging
                if rem(k, log_decim) == 0 || k == total_steps
                    log_idx = log_idx + 1;
                    log_time(log_idx) = k * obj.model.tstp;
                    log_q_des(log_idx, :) = obj.model.des.q_pos(:)';
                    log_q_act(log_idx, :) = obj.model.act.q_pos(:)';
                    log_q_vel_des(log_idx, :) = obj.model.des.q_vel(:)';
                    log_q_vel_act(log_idx, :) = obj.model.act.q_vel(:)';
                    log_q_acc_des(log_idx, :) = obj.model.des.q_acc(:)';
                    log_q_acc_act(log_idx, :) = obj.model.act.q_acc(:)';
                    log_tau(log_idx, :) = tau(:)';
                    
                    T_act = robotics.engines.KinematicsEngine.forwardKinematics(obj.model.TI_0, obj.model.kin, obj.model.act.q_pos);
                    log_p_act(log_idx, :) = T_act(1:3, 4)';
                    if ~isempty(obj.model.des.t_pos)
                        log_p_des(log_idx, :) = obj.model.des.t_pos(:)';
                    else
                        T_des = robotics.engines.KinematicsEngine.forwardKinematics(obj.model.TI_0, obj.model.kin, obj.model.des.q_pos);
                        log_p_des(log_idx, :) = T_des(1:3, 4)';
                    end
                end
                
                % Render frame at frame boundary or completion
                if rem(k, steps_per_frame) == 0 || k == total_steps
                    sim_time = k * obj.model.tstp;
                    
                    if ~isinf(speed_scale)
                        target_wall_time = sim_time / speed_scale;
                        current_wall_time = toc;
                        wait_time = target_wall_time - current_wall_time;
                        if wait_time > 0.001
                            pause(wait_time);
                            current_wall_time = toc;
                        end
                    else
                        current_wall_time = toc;
                    end
                    
                    if last_render_wall_time > 0
                        dt_render = current_wall_time - last_render_wall_time;
                        if dt_render > 0
                            instant_fps = 1.0 / dt_render;
                            fps_smoothed = 0.85 * fps_smoothed + 0.15 * instant_fps;
                        end
                    end
                    last_render_wall_time = current_wall_time;
                    
                    if ~isempty(obj.renderer)
                        obj.renderer.updateView(obj.model);
                    end
                    
                    % UI text callbacks throttled to ~20 Hz to avoid IPC overhead
                    if ~isempty(obj.onUpdateLabels) && (current_wall_time - last_label_wall_time >= 0.050 || k == total_steps)
                        last_label_wall_time = current_wall_time;
                        actual_rtf = sim_time / max(current_wall_time, 1e-4);
                        obj.onUpdateLabels(sim_time, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
                    end
                end
            end
            
            % Save finalized telemetry log
            if log_idx > 0
                logData.time = log_time(1:log_idx);
                logData.q_des = log_q_des(1:log_idx, :);
                logData.q_act = log_q_act(1:log_idx, :);
                logData.q_err = logData.q_des - logData.q_act;
                logData.q_vel_des = log_q_vel_des(1:log_idx, :);
                logData.q_vel_act = log_q_vel_act(1:log_idx, :);
                logData.q_acc_des = log_q_acc_des(1:log_idx, :);
                logData.q_acc_act = log_q_acc_act(1:log_idx, :);
                logData.tau = log_tau(1:log_idx, :);
                logData.p_des = log_p_des(1:log_idx, :);
                logData.p_act = log_p_act(1:log_idx, :);
                logData.p_err = sqrt(sum((logData.p_des - logData.p_act).^2, 2));
                logData.robot_name = obj.robot.Name;
                logData.mode = 'Dynamic';
                logData.kin = obj.model.kin;
                logData.dyn = obj.model.dyn;
                logData.ctr = obj.model.ctr;
                obj.model.logData = logData;
            end
            
            % Ensure final frame and labels are drawn at completion
            if ~isempty(obj.renderer)
                obj.renderer.updateView(obj.model);
            end
            if ~isempty(obj.onUpdateLabels)
                obj.onUpdateLabels(obj.model.tfin, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
            end
        end
        
        function runKinematics(obj)
            % RUNKINEMATICS Executes pure kinematic trajectory replay.
            %   Tracks joint trajectories directly without torque/dynamics simulation,
            %   rendering at high framerates locked to real-time wall-clock speed.
            
            obj.model.running_flag = 1;
            obj.model.des.q_pos = obj.model.fin.q_pos;
            
            trjConfig.tstp = obj.model.tstp;
            trjConfig.tfin_trj = obj.model.tfin_trj;
            trjConfig.trj_profile = obj.model.trj_profile;
            
            speed_scale = obj.model.time_const;
            if isempty(speed_scale) || speed_scale <= 0
                speed_scale = 1.0;
            end
            
            target_fps = 100; % High-framerate rendering cap
            total_steps = round(obj.model.tfin_trj / obj.model.tstp);
            
            if isinf(speed_scale)
                steps_per_frame = total_steps;
            else
                dt_frame_real = 1.0 / target_fps;
                dt_frame_sim = dt_frame_real * speed_scale;
                steps_per_frame = max(1, round(dt_frame_sim / obj.model.tstp));
            end
            
            last_render_wall_time = -inf;
            last_label_wall_time = -inf;
            fps_smoothed = target_fps;
            actual_rtf = speed_scale;
            
            % Pre-allocate diagnostic log arrays
            log_decim = max(1, round(0.001 / obj.model.tstp));
            max_log_len = floor(total_steps / log_decim) + 2;
            log_time = zeros(max_log_len, 1);
            log_q_des = zeros(max_log_len, obj.model.kin.n);
            log_q_act = zeros(max_log_len, obj.model.kin.n);
            log_q_vel_act = zeros(max_log_len, obj.model.kin.n);
            log_q_acc_act = zeros(max_log_len, obj.model.kin.n);
            log_p_des = zeros(max_log_len, 3);
            log_p_act = zeros(max_log_len, 3);
            log_idx = 0;
            
            tic;
            for k = 0:total_steps
                if obj.model.running_flag == 0
                    break;
                end
                
                % Trajectory generation updates actual state directly in kinematic mode
                if obj.model.trj_space == 1
                    invConfig.inv_geo_type = obj.model.inv_geo_type;
                    invConfig.robot_model = obj.model.robot_model;
                    invConfig.TI_0 = obj.model.TI_0;
                    invConfig.inv_geo_trn = obj.model.inv_geo_trn;
                    invConfig.kp_inv = obj.model.kp_inv;
                    invConfig.kr_inv = obj.model.kr_inv;
                    invConfig.kp_trn = obj.model.kp_trn;
                    invConfig.kr_trn = obj.model.kr_trn;
                    
                    ini_x = [obj.model.ini.t_pos; obj.model.ini.r_pos];
                    fin_x = [obj.model.fin.t_pos; obj.model.fin.r_pos];
                    
                    [obj.model.act.q_pos, obj.model.act.q_vel, obj.model.act.q_acc, x_pos, ~, ~] = ...
                        robotics.engines.TrajectoryEngine.generateTaskSpaceTrajectory(...
                            trjConfig, obj.robot, invConfig, obj.model.kin, ini_x, fin_x, ...
                            obj.model.act.q_pos, k, obj.model.eulerSet);
                    p_curr = x_pos(1:3);
                else
                    [obj.model.act.q_pos, obj.model.act.q_vel, obj.model.act.q_acc] = ...
                        robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, obj.model.kin, obj.model.ini.q_pos, obj.model.fin.q_pos, k);
                    T_curr = robotics.engines.KinematicsEngine.forwardKinematics(obj.model.TI_0, obj.model.kin, obj.model.act.q_pos);
                    p_curr = T_curr(1:3, 4);
                end
                
                % Telemetry data logging
                if rem(k, log_decim) == 0 || k == total_steps
                    log_idx = log_idx + 1;
                    log_time(log_idx) = k * obj.model.tstp;
                    log_q_des(log_idx, :) = obj.model.act.q_pos(:)';
                    log_q_act(log_idx, :) = obj.model.act.q_pos(:)';
                    log_q_vel_act(log_idx, :) = obj.model.act.q_vel(:)';
                    log_q_acc_act(log_idx, :) = obj.model.act.q_acc(:)';
                    log_p_des(log_idx, :) = p_curr(:)';
                    log_p_act(log_idx, :) = p_curr(:)';
                end
                
                % Render frame at frame boundary or completion
                if rem(k, steps_per_frame) == 0 || k == total_steps
                    sim_time = k * obj.model.tstp;
                    
                    if ~isinf(speed_scale)
                        target_wall_time = sim_time / speed_scale;
                        current_wall_time = toc;
                        wait_time = target_wall_time - current_wall_time;
                        if wait_time > 0.001
                            pause(wait_time);
                            current_wall_time = toc;
                        end
                    else
                        current_wall_time = toc;
                    end
                    
                    if last_render_wall_time > 0
                        dt_render = current_wall_time - last_render_wall_time;
                        if dt_render > 0
                            instant_fps = 1.0 / dt_render;
                            fps_smoothed = 0.85 * fps_smoothed + 0.15 * instant_fps;
                        end
                    end
                    last_render_wall_time = current_wall_time;
                    
                    if ~isempty(obj.renderer)
                        obj.renderer.updateView(obj.model);
                    end
                    
                    % UI text callbacks throttled to ~20 Hz to avoid IPC overhead
                    if ~isempty(obj.onUpdateLabels) && (current_wall_time - last_label_wall_time >= 0.050 || k == total_steps)
                        last_label_wall_time = current_wall_time;
                        actual_rtf = sim_time / max(current_wall_time, 1e-4);
                        obj.onUpdateLabels(sim_time, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
                    end
                end
            end
            
            % Save finalized telemetry log
            if log_idx > 0
                logData.time = log_time(1:log_idx);
                logData.q_des = log_q_des(1:log_idx, :);
                logData.q_act = log_q_act(1:log_idx, :);
                logData.q_err = zeros(size(logData.q_act));
                logData.q_vel_des = log_q_vel_act(1:log_idx, :);
                logData.q_vel_act = log_q_vel_act(1:log_idx, :);
                logData.q_acc_des = log_q_acc_act(1:log_idx, :);
                logData.q_acc_act = log_q_acc_act(1:log_idx, :);
                logData.tau = zeros(size(logData.q_act));
                logData.p_des = log_p_des(1:log_idx, :);
                logData.p_act = log_p_act(1:log_idx, :);
                logData.p_err = zeros(log_idx, 1);
                logData.robot_name = obj.robot.Name;
                logData.mode = 'Kinematic';
                logData.kin = obj.model.kin;
                logData.dyn = obj.model.dyn;
                logData.ctr = obj.model.ctr;
                obj.model.logData = logData;
            end
            
            % Ensure final frame and labels are drawn at completion
            if ~isempty(obj.renderer)
                obj.renderer.updateView(obj.model);
            end
            if ~isempty(obj.onUpdateLabels)
                obj.onUpdateLabels(obj.model.tfin_trj, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
            end
        end
    end
end
