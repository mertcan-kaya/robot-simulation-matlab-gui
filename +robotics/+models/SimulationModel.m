classdef SimulationModel < handle
    % SIMULATIONMODEL Strict data model holding all physics and UI state
    % Free from all UI/App Designer dependencies
    properties
        % Configurations
        robot_model = 1;
        high_quality = 1;
        ee_att = 1;
        task_mode = 0;
        running_flag = 0;
        coord_frame_on = 1;
        ghost_on = 1;
        line_on = 0;
        
        % Simulation Timing
        tstp = 0.001;
        tfin_trj = 0;
        trj_profile = 3;
        trj_profile_text = 'Quintic';
        time_const = 0.1;
        tfin = 0;
        
        % Kinematics and Dynamics Structs
        TI_0 = eye(4);
        kin;
        dyn;
        ctr;
        
        % State Structs (Positions, Velocities, etc.)
        ini;
        fin;
        act;
        des;
        fbk;
        ref;
        
        % End Effector special properties
        % End Effector special properties
        qEIni = 0;
        qEDes = 0;
        
        % Inverse Kinematics Parameters
        inv_geo_type = 2; % 0: alg, 1: num, 2: hybrid
        inv_geo_trn = 0;
        kp_inv = 0.23;
        kr_inv = 0.015;
        kp_trn = 0.003;
        kr_trn = 0.0001;
    end
    events
        ModelUpdated
    end
    
    methods
        function obj = SimulationModel()
            % Constructor - initialize strict data structures
            obj.kin = struct();
            obj.kin.g0 = -[0;0;9.81];
            
            obj.dyn = struct();
            obj.dyn.spring_on = 0;
            obj.dyn.friction_on = 0;
            
            obj.ini = struct();
            obj.fin = struct();
            obj.act = struct();
            obj.des = struct();
            obj.fbk = struct();
            obj.ctr = struct();
            obj.ref = struct();
        end
        
        function notifyUpdate(obj)
            % Helper to trigger the UI update event
            notify(obj, 'ModelUpdated');
        end
    end
end




