classdef SimulationController < handle
    % SIMULATIONCONTROLLER The core MVC Controller for the robot simulation.
    % It coordinates between the SimulationModel (state), the RobotModel (math),
    % and the RobotRenderer (graphics), fully decoupling them from the GUI.
    
    properties
        model
        robot
        renderer
        
        % Callbacks for UI updates (to prevent hardcoding GUI elements)
        onUpdateLabels
    end
    
    methods
        function obj = SimulationController(axesHandle)
            obj.model = robotics.models.SimulationModel();
            obj.renderer = robotics.graphics.RobotRenderer(axesHandle);
        end
        
        function initRobot(obj)
            % Instantiates the OOP Robot physics and loads the 3D meshes
            obj.robot = robotics.models.RobotFactory.create(obj.model.robot_model);
            
            % Initialize kinematics and dynamics from the OOP robot
            if obj.model.robot_model ~= 0
                new_kin = obj.robot.getKinematicParameters(obj.model.ee_att);
                fields = fieldnames(new_kin);
                if ~isfield(obj.model, 'kin')
                    obj.model.kin = struct();
                end
                for i = 1:length(fields)
                    obj.model.kin.(fields{i}) = new_kin.(fields{i});
                end
            end
            
            if ~isfield(obj.model.kin, 'g0')
                obj.model.kin.g0 = -[0;0;9.81];
            end
            
            % Compute ri_j manually (previously done inside MainApp startupFcn)
            obj.model.kin.ri_j = zeros(3,1,obj.model.kin.n+2);
            for j = 1:obj.model.kin.n+1
                obj.model.kin.ri_j(:,:,j) = robotics.math.getri_j_vec(obj.model.kin.alpha_j(j), obj.model.kin.a_j(j), obj.model.kin.d_j(j));
            end
            if ismethod(obj.robot, 'getInertialParameters')
                new_dyn = obj.robot.getInertialParameters();
                fields = fieldnames(new_dyn);
                if ~isfield(obj.model, 'dyn')
                    obj.model.dyn = struct();
                end
                for i = 1:length(fields)
                    obj.model.dyn.(fields{i}) = new_dyn.(fields{i});
                end
            end
            % Ensure state vectors match the current robot DoF
            n = obj.model.kin.n;
            
            if isempty(obj.model.ini) || ~isfield(obj.model.ini, 'q_pos') || length(obj.model.ini.q_pos) ~= n
                obj.model.ini.q_pos = zeros(n, 1);
            end
            if isempty(obj.model.fin) || ~isfield(obj.model.fin, 'q_pos') || length(obj.model.fin.q_pos) ~= n
                obj.model.fin.q_pos = zeros(n, 1);
            end
            
            obj.model.act.q_pos = obj.model.ini.q_pos;
            obj.model.act.q_vel = zeros(n, 1);
            obj.model.act.q_acc = zeros(n, 1);
            
            obj.model.des.q_pos = obj.model.ini.q_pos;
            obj.model.des.q_vel = zeros(n, 1);
            obj.model.des.q_acc = zeros(n, 1);
            
            obj.model.fbk.q_pos = obj.model.ini.q_pos;
            obj.model.fbk.q_vel = zeros(n, 1);
            
            % Initialize transformation matrices and orientations for ini and fin
            obj.model.ini.Ti = robotics.engines.KinematicsEngine.getTransMatrix(obj.model.TI_0, obj.model.kin.a_j, obj.model.kin.alpha_j, obj.model.kin.d_j, obj.model.kin.theta_O_j, obj.model.kin.j_type, obj.model.ini.q_pos);
            [obj.model.ini.Re, obj.model.ini.t_pos] = robotics.math.SE3_SO3R3(obj.model.ini.Ti(:,:,n+2));
            
            obj.model.fin.Ti = robotics.engines.KinematicsEngine.getTransMatrix(obj.model.TI_0, obj.model.kin.a_j, obj.model.kin.alpha_j, obj.model.kin.d_j, obj.model.kin.theta_O_j, obj.model.kin.j_type, obj.model.fin.q_pos);
            [obj.model.fin.Re, obj.model.fin.t_pos] = robotics.math.SE3_SO3R3(obj.model.fin.Ti(:,:,n+2));
            
            % Update default control parameters from the robot object
            if ~isfield(obj.model.ctr, 'algo')
                obj.model.ctr.algo = 1;
            end
            algo = obj.model.ctr.algo;
            default_ctr = obj.robot.getDefaultControlParams(algo);
            
            % Merge default_ctr into obj.model.ctr while keeping algo
            fields = fieldnames(default_ctr);
            for i = 1:length(fields)
                obj.model.ctr.(fields{i}) = default_ctr.(fields{i});
            end
            
            obj.renderer.loadMeshes(obj.model.robot_model, ...
                obj.model.high_quality, obj.model.ee_att, ...
                obj.model.coord_frame_on, obj.model.ghost_on, ...
                obj.model.line_on, obj.model.task_mode, ...
                obj.model.running_flag, obj.model.trj_profile);
                
            obj.updateView();
        end
        
        function updateView(obj)
            % Forces a graphic update based on the current model state
            obj.renderer.updateView(obj.model);
        end
        
        function runSimulation(obj)
            % The main simulation loop extracted from run_dynamic.m
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
                
                % Graphics update loop
                if sim_time > real_time*obj.model.time_const
                    obj.renderer.updateView(obj.model);
                    
                    % UI callbacks (if registered by the View)
                    if ~isempty(obj.onUpdateLabels)
                        obj.onUpdateLabels(k*obj.model.tstp, obj.model.act.q_pos, obj.model.kin.n);
                    end
                end
                
            end
        end
        
        function runKinematics(obj)
            % The kinematic simulation loop extracted from run_kinematic.m
            obj.model.running_flag = 1;
            obj.model.des.q_pos = obj.model.fin.q_pos;
            
            trjConfig.tstp = obj.model.tstp;
            trjConfig.tfin_trj = obj.model.tfin_trj;
            trjConfig.trj_profile = obj.model.trj_profile;
            
            sim_time = 0;
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
                
                % Graphics update loop
                if sim_time > real_time*obj.model.time_const
                    obj.renderer.updateView(obj.model);
                    
                    % UI callbacks (if registered by the View)
                    if ~isempty(obj.onUpdateLabels)
                        obj.onUpdateLabels(k*obj.model.tstp, obj.model.act.q_pos, obj.model.kin.n);
                    end
                end
            end
        end
        
        function stopSimulation(obj)
            % Halt the simulation gracefully
            obj.model.running_flag = 0;
            obj.renderer.updateView(obj.model);
        end
        
    end
end


