classdef SimulationController < handle
    % SIMULATIONCONTROLLER The core MVC Controller for the robot simulation.
    % It coordinates between the SimulationModel (state), the RobotModel (math),
    % and the RobotRenderer (graphics), fully decoupling them from the GUI.
    
    properties
        model
        robot
        renderer
        engine
        
        % Callbacks for UI updates (to prevent hardcoding GUI elements)
        onUpdateLabels
    end
    
    methods
        function obj = SimulationController(axesHandle)
            obj.model = robotics.models.SimulationModel();
            obj.renderer = robotics.graphics.RobotRenderer(axesHandle);
            obj.engine = robotics.engines.SimulationEngine(obj.model, [], obj.renderer, []);
        end
        
        function setRobotModel(obj, robotId, eeAtt, customParams)
            obj.model.robot_model = robotId;
            obj.model.ee_att = eeAtt;
            
            % If it's a custom robot, apply its manual properties
            if robotId == 0 && nargin > 3
                obj.model.customParams = customParams;
            end
            
            obj.initRobot();
            obj.model.notifyRobotChanged();
        end
        
        function toggleSprings(obj, isOn)
            obj.model.dyn.spring_on = isOn;
            obj.model.notifyRobotChanged();
        end
        
        function toggleFriction(obj, isOn)
            obj.model.dyn.friction_on = isOn;
            obj.model.notifyRobotChanged();
        end
        
        function updateForwardKinematics(obj, targetType, eulerSet)
            if strcmp(targetType, 'ini')
                q_pos = obj.model.ini.q_pos;
            else
                q_pos = obj.model.fin.q_pos;
            end
            
            Ti = zeros(4,4,obj.model.kin.n+2);
            Ti(:,:,1) = eye(4);
            for j = 1:obj.model.kin.n
                Ti(:,:,j+1) = Ti(:,:,j) * robotics.math.SO3R3_SE3(robotics.math.getRi_j(obj.model.kin.alpha_j(j), obj.model.kin.theta_O_j(j)+q_pos(j)), obj.model.kin.ri_j(:,:,j));
            end
            Ti(:,:,obj.model.kin.n+2) = Ti(:,:,obj.model.kin.n+1) * robotics.math.SO3R3_SE3(robotics.math.getRi_j(obj.model.kin.alpha_j(obj.model.kin.n+1), obj.model.kin.theta_O_j(obj.model.kin.n+1)), obj.model.kin.ri_j(:,:,obj.model.kin.n+1));
            
            [Re, t_pos] = robotics.math.SE3_SO3R3(Ti(:,:,obj.model.kin.n+2));
            r_pos = robotics.math.getEulerPosVec(Re, eulerSet);
            
            if strcmp(targetType, 'ini')
                obj.model.ini.Ti = Ti;
                obj.model.ini.Re = Re;
                obj.model.ini.t_pos = t_pos;
                obj.model.ini.r_pos = r_pos;
            else
                obj.model.fin.Ti = Ti;
                obj.model.fin.Re = Re;
                obj.model.fin.t_pos = t_pos;
                obj.model.fin.r_pos = r_pos;
            end
        end

        function setJointPosition(obj, targetType, jointIndex, value, eulerSet)
            if strcmp(targetType, 'ini')
                obj.model.ini.q_pos(jointIndex) = value;
            else
                obj.model.fin.q_pos(jointIndex) = value;
            end
            obj.updateForwardKinematics(targetType, eulerSet);
            obj.model.notifyTargetUpdated();
        end

        function updateFinTimeTrj(obj, prcnt)
            obj.model.tfin_trj = robotics.engines.TrajectoryEngine.computeTime(...
                obj.model.ini.q_pos, obj.model.fin.q_pos, prcnt, obj.model.trj_profile, ...
                obj.model.kin.q_velLim, obj.model.kin.q_accLim);
            obj.model.notifyTargetUpdated();
        end
        
        function setTargetPosition(obj, targetType, xyzValues, eulerValues, eulerSet)
            % Inverse kinematics update logic
            Re_ref = robotics.math.getRotMatfromEA(eulerValues, eulerSet);
            t_pos_ref = xyzValues;
            
            invGeoConfig.inv_geo_type = obj.model.inv_geo_type;
            invGeoConfig.robot_model = obj.model.robot_model;
            invGeoConfig.TI_0 = obj.model.TI_0;
            invGeoConfig.inv_geo_trn = obj.model.inv_geo_trn;
            invGeoConfig.kp_inv = obj.model.kp_inv;
            invGeoConfig.kr_inv = obj.model.kr_inv;
            invGeoConfig.kp_trn = obj.model.kp_trn;
            invGeoConfig.kr_trn = obj.model.kr_trn;
            
            if strcmp(targetType, 'ini')
                qDes = robotics.engines.KinematicsEngine.inverseKinematics(obj.robot, invGeoConfig, obj.model.kin, Re_ref, t_pos_ref, obj.model.ini.q_pos);
                obj.model.ini.q_pos = qDes;
            else
                qDes = robotics.engines.KinematicsEngine.inverseKinematics(obj.robot, invGeoConfig, obj.model.kin, Re_ref, t_pos_ref, obj.model.fin.q_pos);
                obj.model.fin.q_pos = qDes;
            end
            
            % Update kinematics matrices based on new q_pos
            obj.updateForwardKinematics(targetType, eulerSet);
            obj.model.notifyTargetUpdated();
        end
        
        function initRobot(obj)
            % Instantiates the OOP Robot physics and loads the 3D meshes
            if obj.model.robot_model == 0 && ~isempty(obj.model.customParams)
                obj.robot = robotics.models.RobotFactory.create(obj.model.robot_model, obj.model.customParams);
            else
                obj.robot = robotics.models.RobotFactory.create(obj.model.robot_model);
            end
            
            % Initialize kinematics and dynamics from the OOP robot
            new_kin = obj.robot.getKinematicParameters(obj.model.ee_att);
            fields = fieldnames(new_kin);
            if isempty(obj.model.kin)
                obj.model.kin = struct();
            end
            for i = 1:length(fields)
                obj.model.kin.(fields{i}) = new_kin.(fields{i});
            end
            
            if ~isfield(obj.model.kin, 'g0')
                obj.model.kin.g0 = -[0;0;9.81];
            end
            
            % Compute ri_j manually (previously done inside MainApp startupFcn)
            obj.model.kin.ri_j = zeros(3,1,obj.model.kin.n+2);
            for j = 1:obj.model.kin.n+1
                obj.model.kin.ri_j(:,:,j) = robotics.math.getri_j_vec(obj.model.kin.alpha_j(j), obj.model.kin.a_j(j), obj.model.kin.d_j(j));
            end
            
            if ~isfield(obj.model.kin, 'r_posLim')
                obj.model.kin.r_posLim = [-pi pi; -pi pi; -pi pi];
            end
            if ~isfield(obj.model.kin, 'r_velLim')
                obj.model.kin.r_velLim = [-pi/4 pi/4];
            end
            if ~isfield(obj.model.kin, 'r_accLim')
                obj.model.kin.r_accLim = [-pi/8 pi/8];
            end
            if ~isfield(obj.model.kin, 't_posLim')
                obj.model.kin.t_posLim = [-1 1; -1 1; -1 1];
            end
            if ~isfield(obj.model.kin, 'q_posLim')
                obj.model.kin.q_posLim = zeros(obj.model.kin.n,2);
                obj.model.kin.q_velLim = zeros(obj.model.kin.n,2);
                obj.model.kin.q_accLim = zeros(obj.model.kin.n,2);
                for j = 1:obj.model.kin.n
                    if obj.model.kin.j_type(j) == 1
                        obj.model.kin.q_posLim(j,:) = [-pi, pi];
                        obj.model.kin.q_velLim(j,:) = [-pi, pi];
                        obj.model.kin.q_accLim(j,:) = [-pi, pi];
                    else
                        obj.model.kin.q_posLim(j,:) = [-1, 1];
                        obj.model.kin.q_velLim(j,:) = [-1, 1];
                        obj.model.kin.q_accLim(j,:) = [-1, 1];
                    end
                end
            end
            if ismethod(obj.robot, 'getInertialParameters')
                new_dyn = obj.robot.getInertialParameters();
                fields = fieldnames(new_dyn);
                if isempty(obj.model.dyn)
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
            obj.model.ini.r_pos = robotics.math.getEulerPosVec(obj.model.ini.Re, 1); % Assume eulerSet 1 (ZYZ) by default
            
            obj.model.fin.Ti = robotics.engines.KinematicsEngine.getTransMatrix(obj.model.TI_0, obj.model.kin.a_j, obj.model.kin.alpha_j, obj.model.kin.d_j, obj.model.kin.theta_O_j, obj.model.kin.j_type, obj.model.fin.q_pos);
            [obj.model.fin.Re, obj.model.fin.t_pos] = robotics.math.SE3_SO3R3(obj.model.fin.Ti(:,:,n+2));
            obj.model.fin.r_pos = robotics.math.getEulerPosVec(obj.model.fin.Re, 1);
            
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
            obj.engine.robot = obj.robot;
            obj.engine.onUpdateLabels = obj.onUpdateLabels;
            obj.engine.runSimulation();
        end
        
        function runKinematics(obj)
            obj.engine.robot = obj.robot;
            obj.engine.onUpdateLabels = obj.onUpdateLabels;
            obj.engine.runKinematics();
        end
        
        function stopSimulation(obj)

            % Halt the simulation gracefully
            obj.model.running_flag = 0;
            obj.renderer.updateView(obj.model);
        end
        
    end
end


