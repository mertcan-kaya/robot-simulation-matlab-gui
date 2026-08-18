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
            %       renderer       - RobotRenderer instance (3D graphics pipeline)
            %       onUpdateLabels - function_handle: UI callback for stats (time, FPS, RTF)
            
            obj.model = model;
            obj.robot = robot;
            obj.renderer = renderer;
            obj.onUpdateLabels = onUpdateLabels;
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
            dt_frame_real = 1.0 / target_fps;
            dt_frame_sim = dt_frame_real * speed_scale;
            steps_per_frame = max(1, round(dt_frame_sim / obj.model.tstp));
            total_steps = round(obj.model.tfin / obj.model.tstp);
            
            last_render_wall_time = -inf;
            last_label_wall_time = -inf;
            fps_smoothed = target_fps;
            actual_rtf = speed_scale;
            
            tic;
            for k = 0:total_steps
                
                if obj.model.running_flag == 0
                    break;
                end
                
                % Low frequency control loop
                if rem(k, d) == 0
                    obj.model.fbk.q_vel = obj.model.act.q_vel;
                    obj.model.fbk.q_pos = obj.model.act.q_pos;
                    
                    [obj.model.des.q_pos, obj.model.des.q_vel, obj.model.des.q_acc] = ...
                        robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, obj.model.kin, obj.model.ini.q_pos, obj.model.fin.q_pos, k);
                end
                
                tau = robotics.engines.ControlEngine.computeTorque(obj.model.ctr, obj.model.kin, obj.model.dyn, obj.model.des, obj.model.fbk);
                
                % High frequency physics loop
                obj.model.act.q_acc = robotics.engines.DynamicsEngine.forwardDynamics(obj.robot, obj.model.kin, obj.model.dyn, tau, obj.model.act.q_pos, obj.model.act.q_vel);
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
                
                % Render frame at frame boundary or completion
                if rem(k, steps_per_frame) == 0 || k == total_steps
                    sim_time = k * obj.model.tstp;
                    target_wall_time = sim_time / speed_scale;
                    current_wall_time = toc;
                    
                    wait_time = target_wall_time - current_wall_time;
                    if wait_time > 0.001
                        pause(wait_time);
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
                    
                    obj.renderer.updateView(obj.model);
                    
                    % UI text callbacks throttled to ~20 Hz to avoid IPC overhead
                    if ~isempty(obj.onUpdateLabels) && (current_wall_time - last_label_wall_time >= 0.050 || k == total_steps)
                        last_label_wall_time = current_wall_time;
                        actual_rtf = sim_time / max(current_wall_time, 1e-4);
                        obj.onUpdateLabels(sim_time, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
                    end
                end
            end
            
            % Ensure final frame and labels are drawn at completion
            obj.renderer.updateView(obj.model);
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
            dt_frame_real = 1.0 / target_fps;
            dt_frame_sim = dt_frame_real * speed_scale;
            steps_per_frame = max(1, round(dt_frame_sim / obj.model.tstp));
            total_steps = round(obj.model.tfin_trj / obj.model.tstp);
            
            last_render_wall_time = -inf;
            last_label_wall_time = -inf;
            fps_smoothed = target_fps;
            actual_rtf = speed_scale;
            
            tic;
            for k = 0:total_steps
                if obj.model.running_flag == 0
                    break;
                end
                
                % Trajectory generation updates actual state directly in kinematic mode
                [obj.model.act.q_pos, obj.model.act.q_vel, obj.model.act.q_acc] = ...
                    robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, obj.model.kin, obj.model.ini.q_pos, obj.model.fin.q_pos, k);
                
                % Render frame at frame boundary or completion
                if rem(k, steps_per_frame) == 0 || k == total_steps
                    sim_time = k * obj.model.tstp;
                    target_wall_time = sim_time / speed_scale;
                    current_wall_time = toc;
                    
                    wait_time = target_wall_time - current_wall_time;
                    if wait_time > 0.001
                        pause(wait_time);
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
                    
                    obj.renderer.updateView(obj.model);
                    
                    % UI text callbacks throttled to ~20 Hz to avoid IPC overhead
                    if ~isempty(obj.onUpdateLabels) && (current_wall_time - last_label_wall_time >= 0.050 || k == total_steps)
                        last_label_wall_time = current_wall_time;
                        actual_rtf = sim_time / max(current_wall_time, 1e-4);
                        obj.onUpdateLabels(sim_time, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
                    end
                end
            end
            
            % Ensure final frame and labels are drawn at completion
            obj.renderer.updateView(obj.model);
            if ~isempty(obj.onUpdateLabels)
                obj.onUpdateLabels(obj.model.tfin_trj, obj.model.act.q_pos, obj.model.kin.n, fps_smoothed, actual_rtf);
            end
        end
    end
end
