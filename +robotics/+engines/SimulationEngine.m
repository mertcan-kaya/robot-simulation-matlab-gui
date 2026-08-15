classdef SimulationEngine < handle
    % SIMULATIONENGINE Extracts the heavy simulation loops from the Controller.
    % Handles the time-stepping execution for runSimulation and runKinematics.
    
    properties
        model
        robot
        renderer
        onUpdateLabels
    end
    
    methods
        function obj = SimulationEngine(model, robot, renderer, onUpdateLabels)
            obj.model = model;
            obj.robot = robot;
            obj.renderer = renderer;
            obj.onUpdateLabels = onUpdateLabels;
        end
        
        function runSimulation(obj)
            % The main simulation loop extracted from SimulationController
            obj.model.running_flag = 1;
            
            d = round(obj.model.ctr.tcyc/obj.model.tstp);
            obj.model.ctr.errsum = zeros(obj.model.kin.n,1);
            
            obj.model.act.q_pos = obj.model.ini.q_pos;
            obj.model.act.q_vel = zeros(obj.model.kin.n,1);
            obj.model.act.q_acc = zeros(obj.model.kin.n,1);
            
            trjConfig.tstp = obj.model.tstp;
            trjConfig.tfin_trj = obj.model.tfin_trj;
            trjConfig.trj_profile = obj.model.trj_profile;
            
            sim_time = 0;
            last_render_real_time = -inf;
            tic
            
            for k = 0:round(obj.model.tfin/obj.model.tstp)
                
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
                
                sim_time = sim_time + obj.model.tstp;
                real_time = toc;
                
                % Graphics update loop (throttled to ~60 FPS max to avoid choking the UI event queue)
                if ((real_time - last_render_real_time >= 0.016) && (sim_time >= real_time*obj.model.time_const))
                    last_render_real_time = real_time;
                    obj.renderer.updateView(obj.model);
                    
                    % UI callbacks (if registered by the View)
                    if ~isempty(obj.onUpdateLabels)
                        obj.onUpdateLabels(k*obj.model.tstp, obj.model.act.q_pos, obj.model.kin.n);
                    end
                end
                
            end
            
            % Ensure final frame and labels are drawn at completion
            obj.renderer.updateView(obj.model);
            if ~isempty(obj.onUpdateLabels)
                obj.onUpdateLabels(obj.model.tfin, obj.model.act.q_pos, obj.model.kin.n);
            end
        end
        
        function runKinematics(obj)
            % The kinematic simulation loop
            obj.model.running_flag = 1;
            obj.model.des.q_pos = obj.model.fin.q_pos;
            
            trjConfig.tstp = obj.model.tstp;
            trjConfig.tfin_trj = obj.model.tfin_trj;
            trjConfig.trj_profile = obj.model.trj_profile;
            
            sim_time = 0;
            last_render_real_time = -inf;
            tic
            
            for k = 0:round(obj.model.tfin_trj/obj.model.tstp)
                if obj.model.running_flag == 0
                    break;
                end
                
                % Trajectory generation updates actual state directly in kinematic mode
                [obj.model.act.q_pos, obj.model.act.q_vel, obj.model.act.q_acc] = ...
                    robotics.engines.TrajectoryEngine.generateTrajectory(trjConfig, obj.model.kin, obj.model.ini.q_pos, obj.model.fin.q_pos, k);
                    
                sim_time = sim_time + obj.model.tstp;
                real_time = toc;
                
                % Graphics update loop (throttled to ~60 FPS max to avoid choking the UI event queue)
                if ((real_time - last_render_real_time >= 0.016) && (sim_time >= real_time*obj.model.time_const))
                    last_render_real_time = real_time;
                    obj.renderer.updateView(obj.model);
                    
                    % UI callbacks (if registered by the View)
                    if ~isempty(obj.onUpdateLabels)
                        obj.onUpdateLabels(k*obj.model.tstp, obj.model.act.q_pos, obj.model.kin.n);
                    end
                end
            end
            
            % Ensure final frame and labels are drawn at completion
            obj.renderer.updateView(obj.model);
            if ~isempty(obj.onUpdateLabels)
                obj.onUpdateLabels(obj.model.tfin_trj, obj.model.act.q_pos, obj.model.kin.n);
            end
        end
    end
end
