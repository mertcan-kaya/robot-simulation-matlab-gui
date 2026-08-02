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
        qEIni;
        qEDes;
    end
    
    methods
        function obj = SimulationModel()
            % Constructor (empty for now)
        end
    end
end
