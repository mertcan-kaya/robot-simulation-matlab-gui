classdef SimulationViewModel < handle
    %SIMULATIONVIEWMODEL Exposes pre-formatted strings and simplified properties 
    % for the UI to bind to, abstracting away the SimulationModel details.
    
    properties (SetAccess = private)
        model (1,1) robotics.models.SimulationModel
    end
    
    properties (Dependent)
        % Time and Status
        CurrentTimeText
        TaskErrorText
        
        % Formatted State Arrays
        ActualJointPositionsText
        
        % Dynamic limits based on current robot
        Joint1LimitText
        Joint2LimitText
        Joint3LimitText
        Joint4LimitText
        Joint5LimitText
        Joint6LimitText
        Joint7LimitText
    end
    
    methods
        function obj = SimulationViewModel(model)
            obj.model = model;
        end
        
        function text = get.CurrentTimeText(obj)
            if isfield(obj.model.ctr, 'time') && ~isempty(obj.model.ctr.time)
                t = obj.model.ctr.time;
            else
                t = 0;
            end
            text = sprintf('%.2f sec', t);
        end
        
        function text = get.TaskErrorText(obj)
            if isfield(obj.model.ctr, 'err_t') && numel(obj.model.ctr.err_t) >= 3
                text = sprintf('e_task = [%.2f, %.2f, %.2f] mm', ...
                    obj.model.ctr.err_t(1)*1000, ...
                    obj.model.ctr.err_t(2)*1000, ...
                    obj.model.ctr.err_t(3)*1000);
            else
                text = 'e_task = [0.00, 0.00, 0.00] mm';
            end
        end
        
        function text = get.ActualJointPositionsText(obj)
            if isempty(obj.model.act) || ~isfield(obj.model.act, 'q_pos') || isempty(obj.model.act.q_pos)
                text = 'qact=[] deg';
                return;
            end
            q_deg = rad2deg(obj.model.act.q_pos(:));
            str_vals = arrayfun(@(v) sprintf('%.1f', v), q_deg, 'UniformOutput', false);
            text = sprintf('q_act=[%s] deg', strjoin(str_vals, ', '));
        end
        
        % Generic limit getter
        function text = getJointLimitText(obj, jointIdx)
            if jointIdx <= obj.model.kin.n
                text = sprintf('Limits: [%.0f, %.0f]', ...
                    rad2deg(obj.model.kin.q_posLim(jointIdx,1)), ...
                    rad2deg(obj.model.kin.q_posLim(jointIdx,2)));
            else
                text = 'Limits: [0, 0]';
            end
        end
        
        function text = get.Joint1LimitText(obj); text = obj.getJointLimitText(1); end
        function text = get.Joint2LimitText(obj); text = obj.getJointLimitText(2); end
        function text = get.Joint3LimitText(obj); text = obj.getJointLimitText(3); end
        function text = get.Joint4LimitText(obj); text = obj.getJointLimitText(4); end
        function text = get.Joint5LimitText(obj); text = obj.getJointLimitText(5); end
        function text = get.Joint6LimitText(obj); text = obj.getJointLimitText(6); end
        function text = get.Joint7LimitText(obj); text = obj.getJointLimitText(7); end
        
        function data = getGainTableData(obj)
            n = obj.model.kin.n;
            data = cell(n, 3);
            if isfield(obj.model.ctr, 'algo') && obj.model.ctr.algo == 1
                % Computed Torque Control (IDC)
                if isfield(obj.model.ctr, 'Kp_jnt_idc')
                    for i = 1:n
                        data{i, 1} = obj.model.ctr.Kp_jnt_idc(i);
                        data{i, 2} = obj.model.ctr.Ki_jnt_idc(i);
                        data{i, 3} = obj.model.ctr.Kd_jnt_idc(i);
                    end
                end
            else
                % PID Control
                if isfield(obj.model.ctr, 'Kp_jnt_pid')
                    for i = 1:n
                        data{i, 1} = obj.model.ctr.Kp_jnt_pid(i);
                        data{i, 2} = obj.model.ctr.Ki_jnt_pid(i);
                        data{i, 3} = obj.model.ctr.Kd_jnt_pid(i);
                    end
                end
            end
        end
        
        function rowNames = getGainTableRowNames(obj)
            n = obj.model.kin.n;
            rowNames = cell(n, 1);
            for i = 1:n
                rowNames{i} = num2str(i);
            end
        end
        
    end
end
