function updateControlParams(app)
    % Decoupled: Fetch default control parameters from the Robot object
    robot = RobotFactory.create(app.robot_model);
    default_ctr = robot.getDefaultControlParams(app.ctr.algo);
    
    app.ctr.tcyc = default_ctr.tcyc;
    
    if app.ctr.algo == 1
        % Computed Torque Control (IDC)
        if isfield(default_ctr, 'Kp_jnt_idc')
            app.ctr.Kp_jnt_idc = default_ctr.Kp_jnt_idc;
            app.ctr.Ki_jnt_idc = default_ctr.Ki_jnt_idc;
            app.ctr.Kd_jnt_idc = default_ctr.Kd_jnt_idc;
            
            % Update GUI table
            n = app.kin.n;
            tableData = cell(n, 3);
            for i = 1:n
                tableData{i, 1} = app.ctr.Kp_jnt_idc(i);
                tableData{i, 2} = app.ctr.Ki_jnt_idc(i);
                tableData{i, 3} = app.ctr.Kd_jnt_idc(i);
            end
            app.GainTable.Data = tableData;
        end
    else
        % PID Control
        app.ctr.Kp_jnt_pid = default_ctr.Kp_jnt_pid;
        app.ctr.Ki_jnt_pid = default_ctr.Ki_jnt_pid;
        app.ctr.Kd_jnt_pid = default_ctr.Kd_jnt_pid;
        
        % Update GUI table
        n = app.kin.n;
        tableData = cell(n, 3);
        for i = 1:n
            tableData{i, 1} = app.ctr.Kp_jnt_pid(i);
            tableData{i, 2} = app.ctr.Ki_jnt_pid(i);
            tableData{i, 3} = app.ctr.Kd_jnt_pid(i);
        end
        app.GainTable.Data = tableData;
    end
    
    app.CycleEditField.Value = app.ctr.tcyc;
    app.CompGravityCheckBox.Value = app.ctr.comp_grv;
    
    % Update table row names based on joints
    n = app.kin.n;
    rowNames = cell(n, 1);
    for i = 1:n
        rowNames{i} = num2str(i);
    end
    app.GainTable.RowName = rowNames;
    
    app.GainTable.ColumnEditable = logical([1 1 1]);
    app.GainTable.ColumnWidth = {65,65,65};
end