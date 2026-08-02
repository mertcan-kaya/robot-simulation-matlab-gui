function updateControlParams(app)
    % Decoupled: Fetch default control parameters from the Robot object
    if ~isfield(app.controller.model.ctr, 'algo')
        app.controller.model.ctr.algo = 1;
    end
    algo = app.controller.model.ctr.algo;
    robot_model = app.controller.model.robot_model;
    
    if robot_model == 0
        return; % Custom robot, no default parameters
    end
    
    robot = RobotFactory.create(robot_model);
    default_ctr = robot.getDefaultControlParams(algo);
    
    % Merge default_ctr into app.controller.model.ctr
    fields = fieldnames(default_ctr);
    for i = 1:length(fields)
        app.controller.model.ctr.(fields{i}) = default_ctr.(fields{i});
    end
    
    app.controller.model.ctr.tcyc = default_ctr.tcyc;
    
    if algo == 1
        % Computed Torque Control (IDC)
        if isfield(default_ctr, 'Kp_jnt_idc')
            % Update GUI table
            n = app.controller.model.kin.n;
            tableData = cell(n, 3);
            for i = 1:n
                tableData{i, 1} = app.controller.model.ctr.Kp_jnt_idc(i);
                tableData{i, 2} = app.controller.model.ctr.Ki_jnt_idc(i);
                tableData{i, 3} = app.controller.model.ctr.Kd_jnt_idc(i);
            end
            app.GainTable.Data = tableData;
        end
    else
        % PID Control
        % Update GUI table
        n = app.controller.model.kin.n;
        tableData = cell(n, 3);
        for i = 1:n
            tableData{i, 1} = app.controller.model.ctr.Kp_jnt_pid(i);
            tableData{i, 2} = app.controller.model.ctr.Ki_jnt_pid(i);
            tableData{i, 3} = app.controller.model.ctr.Kd_jnt_pid(i);
        end
        app.GainTable.Data = tableData;
    end
    
    app.CycleEditField.Value = app.controller.model.ctr.tcyc;
    
    if isfield(app.controller.model.ctr, 'comp_grv')
        app.CompGravityCheckBox.Value = app.controller.model.ctr.comp_grv;
    end
    
    % Update table row names based on joints
    n = app.controller.model.kin.n;
    rowNames = cell(n, 1);
    for i = 1:n
        rowNames{i} = num2str(i);
    end
    app.GainTable.RowName = rowNames;
    
    app.GainTable.ColumnEditable = logical([1 1 1]);
    app.GainTable.ColumnWidth = {65,65,65};
end