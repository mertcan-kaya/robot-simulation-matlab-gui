function updateControlParams(app)

    % Stäubli R160L
    app.ctr.tcyc_5 = 0.004;

    app.ctr.Kp_jnt_pid_5 = [1000;2000;2000;120;100;50];
    app.ctr.Ki_jnt_pid_5 = [0;0;0;0;0;0];
    app.ctr.Kd_jnt_pid_5 = [30;40;40;10;7.5;5];

    % Stäubli R160
    app.ctr.tcyc_4 = 0.004;

    app.ctr.Kp_jnt_pid_4 = [1000;2000;2000;120;100;50];
    app.ctr.Ki_jnt_pid_4 = [0;0;0;0;0;0];
    app.ctr.Kd_jnt_pid_4 = [30;40;40;10;7.5;5];

    % Unitree Z1
    app.ctr.tcyc_3 = 0.001;

    app.ctr.Kp_jnt_pid_3 = [100;100;100;60;50;25];
    app.ctr.Ki_jnt_pid_3 = [0;0;0;0;0;0];
    app.ctr.Kd_jnt_pid_3 = [10;10;10;10;7.5;5];

    % Universal Robots UR3
    app.ctr.tcyc_2 = 0.008;

    app.ctr.Kp_jnt_pid_2 = [100;100;100;60;50;25];
    app.ctr.Ki_jnt_pid_2 = [0;0;0;0;0;0];
    app.ctr.Kd_jnt_pid_2 = [10;10;10;10;7.5;5];

    % Franka Emika Robot
    app.ctr.tcyc_1 = 0.001;

    app.ctr.Kp_jnt_pid_1 = [100;100;100;100;60;60;60];
    app.ctr.Ki_jnt_pid_1 = [0;0;0;0;0;0;0];
    app.ctr.Kd_jnt_pid_1 = [10;10;10;10;5;5;5];

    app.ctr.Kp_jnt_idc_1 = [150;150;150;150;65;65;65];
    app.ctr.Ki_jnt_idc_1 = [0;0;0;0;0;0;0];
    app.ctr.Kd_jnt_idc_1 = [15;15;15;15;10;10;10];

    % Custom Robot
    app.ctr.tcyc_0 = 0.001;

    app.ctr.Kp_jnt_pid_0 = zeros(7,1);
    app.ctr.Ki_jnt_pid_0 = zeros(7,1);
    app.ctr.Kd_jnt_pid_0 = zeros(7,1);

    if app.robot_model == 5
        app.ctr.tcyc = app.ctr.tcyc_5;
        if app.ctr.algo == 1
        else
            app.GainTable.Data = {app.ctr.Kp_jnt_pid_5(1) app.ctr.Ki_jnt_pid_5(1) app.ctr.Kd_jnt_pid_5(1); ...
                                  app.ctr.Kp_jnt_pid_5(2) app.ctr.Ki_jnt_pid_5(2) app.ctr.Kd_jnt_pid_5(2); ...
                                  app.ctr.Kp_jnt_pid_5(3) app.ctr.Ki_jnt_pid_5(3) app.ctr.Kd_jnt_pid_5(3); ...
                                  app.ctr.Kp_jnt_pid_5(4) app.ctr.Ki_jnt_pid_5(4) app.ctr.Kd_jnt_pid_5(4); ...
                                  app.ctr.Kp_jnt_pid_5(5) app.ctr.Ki_jnt_pid_5(5) app.ctr.Kd_jnt_pid_5(5); ...
                                  app.ctr.Kp_jnt_pid_5(6) app.ctr.Ki_jnt_pid_5(6) app.ctr.Kd_jnt_pid_5(6)};
            app.ctr.Kp_jnt_pid = app.ctr.Kp_jnt_pid_5;
            app.ctr.Ki_jnt_pid = app.ctr.Ki_jnt_pid_5;
            app.ctr.Kd_jnt_pid = app.ctr.Kd_jnt_pid_5;
        end
    elseif app.robot_model == 4
        app.ctr.tcyc = app.ctr.tcyc_4;
        if app.ctr.algo == 1
        else
            app.GainTable.Data = {app.ctr.Kp_jnt_pid_4(1) app.ctr.Ki_jnt_pid_4(1) app.ctr.Kd_jnt_pid_4(1); ...
                                  app.ctr.Kp_jnt_pid_4(2) app.ctr.Ki_jnt_pid_4(2) app.ctr.Kd_jnt_pid_4(2); ...
                                  app.ctr.Kp_jnt_pid_4(3) app.ctr.Ki_jnt_pid_4(3) app.ctr.Kd_jnt_pid_4(3); ...
                                  app.ctr.Kp_jnt_pid_4(4) app.ctr.Ki_jnt_pid_4(4) app.ctr.Kd_jnt_pid_4(4); ...
                                  app.ctr.Kp_jnt_pid_4(5) app.ctr.Ki_jnt_pid_4(5) app.ctr.Kd_jnt_pid_4(5); ...
                                  app.ctr.Kp_jnt_pid_4(6) app.ctr.Ki_jnt_pid_4(6) app.ctr.Kd_jnt_pid_4(6)};
            app.ctr.Kp_jnt_pid = app.ctr.Kp_jnt_pid_4;
            app.ctr.Ki_jnt_pid = app.ctr.Ki_jnt_pid_4;
            app.ctr.Kd_jnt_pid = app.ctr.Kd_jnt_pid_4;
        end
    elseif app.robot_model == 3
        app.ctr.tcyc = app.ctr.tcyc_3;
        if app.ctr.algo == 1
        else
            app.GainTable.Data = {app.ctr.Kp_jnt_pid_3(1) app.ctr.Ki_jnt_pid_3(1) app.ctr.Kd_jnt_pid_3(1); ...
                                  app.ctr.Kp_jnt_pid_3(2) app.ctr.Ki_jnt_pid_3(2) app.ctr.Kd_jnt_pid_3(2); ...
                                  app.ctr.Kp_jnt_pid_3(3) app.ctr.Ki_jnt_pid_3(3) app.ctr.Kd_jnt_pid_3(3); ...
                                  app.ctr.Kp_jnt_pid_3(4) app.ctr.Ki_jnt_pid_3(4) app.ctr.Kd_jnt_pid_3(4); ...
                                  app.ctr.Kp_jnt_pid_3(5) app.ctr.Ki_jnt_pid_3(5) app.ctr.Kd_jnt_pid_3(5); ...
                                  app.ctr.Kp_jnt_pid_3(6) app.ctr.Ki_jnt_pid_3(6) app.ctr.Kd_jnt_pid_3(6)};
            app.ctr.Kp_jnt_pid = app.ctr.Kp_jnt_pid_3;
            app.ctr.Ki_jnt_pid = app.ctr.Ki_jnt_pid_3;
            app.ctr.Kd_jnt_pid = app.ctr.Kd_jnt_pid_3;
        end
    elseif app.robot_model == 2
        app.ctr.tcyc = app.ctr.tcyc_2;
        if app.ctr.algo == 1
        else
            app.GainTable.Data = {app.ctr.Kp_jnt_pid_2(1) app.ctr.Ki_jnt_pid_2(1) app.ctr.Kd_jnt_pid_2(1); ...
                                  app.ctr.Kp_jnt_pid_2(2) app.ctr.Ki_jnt_pid_2(2) app.ctr.Kd_jnt_pid_2(2); ...
                                  app.ctr.Kp_jnt_pid_2(3) app.ctr.Ki_jnt_pid_2(3) app.ctr.Kd_jnt_pid_2(3); ...
                                  app.ctr.Kp_jnt_pid_2(4) app.ctr.Ki_jnt_pid_2(4) app.ctr.Kd_jnt_pid_2(4); ...
                                  app.ctr.Kp_jnt_pid_2(5) app.ctr.Ki_jnt_pid_2(5) app.ctr.Kd_jnt_pid_2(5); ...
                                  app.ctr.Kp_jnt_pid_2(6) app.ctr.Ki_jnt_pid_2(6) app.ctr.Kd_jnt_pid_2(6)};
            app.ctr.Kp_jnt_pid = app.ctr.Kp_jnt_pid_2;
            app.ctr.Ki_jnt_pid = app.ctr.Ki_jnt_pid_2;
            app.ctr.Kd_jnt_pid = app.ctr.Kd_jnt_pid_2;
        end
    elseif app.robot_model == 1
        app.ctr.tcyc = app.ctr.tcyc_1;
        if app.ctr.algo == 1
            app.GainTable.Data = {app.ctr.Kp_jnt_idc_1(1) app.ctr.Ki_jnt_idc_1(1) app.ctr.Kd_jnt_idc_1(1); ...
                                  app.ctr.Kp_jnt_idc_1(2) app.ctr.Ki_jnt_idc_1(2) app.ctr.Kd_jnt_idc_1(2); ...
                                  app.ctr.Kp_jnt_idc_1(3) app.ctr.Ki_jnt_idc_1(3) app.ctr.Kd_jnt_idc_1(3); ...
                                  app.ctr.Kp_jnt_idc_1(4) app.ctr.Ki_jnt_idc_1(4) app.ctr.Kd_jnt_idc_1(4); ...
                                  app.ctr.Kp_jnt_idc_1(5) app.ctr.Ki_jnt_idc_1(5) app.ctr.Kd_jnt_idc_1(5); ...
                                  app.ctr.Kp_jnt_idc_1(6) app.ctr.Ki_jnt_idc_1(6) app.ctr.Kd_jnt_idc_1(6); ...
                                  app.ctr.Kp_jnt_idc_1(7) app.ctr.Ki_jnt_idc_1(7) app.ctr.Kd_jnt_idc_1(7)};
            app.ctr.Kp_jnt_idc = app.ctr.Kp_jnt_idc_1;
            app.ctr.Ki_jnt_idc = app.ctr.Ki_jnt_idc_1;
            app.ctr.Kd_jnt_idc = app.ctr.Kd_jnt_idc_1;
        else
            app.GainTable.Data = {app.ctr.Kp_jnt_pid_1(1) app.ctr.Ki_jnt_pid_1(1) app.ctr.Kd_jnt_pid_1(1); ...
                                  app.ctr.Kp_jnt_pid_1(2) app.ctr.Ki_jnt_pid_1(2) app.ctr.Kd_jnt_pid_1(2); ...
                                  app.ctr.Kp_jnt_pid_1(3) app.ctr.Ki_jnt_pid_1(3) app.ctr.Kd_jnt_pid_1(3); ...
                                  app.ctr.Kp_jnt_pid_1(4) app.ctr.Ki_jnt_pid_1(4) app.ctr.Kd_jnt_pid_1(4); ...
                                  app.ctr.Kp_jnt_pid_1(5) app.ctr.Ki_jnt_pid_1(5) app.ctr.Kd_jnt_pid_1(5); ...
                                  app.ctr.Kp_jnt_pid_1(6) app.ctr.Ki_jnt_pid_1(6) app.ctr.Kd_jnt_pid_1(6); ...
                                  app.ctr.Kp_jnt_pid_1(7) app.ctr.Ki_jnt_pid_1(7) app.ctr.Kd_jnt_pid_1(7)};
            app.ctr.Kp_jnt_pid = app.ctr.Kp_jnt_pid_1;
            app.ctr.Ki_jnt_pid = app.ctr.Ki_jnt_pid_1;
            app.ctr.Kd_jnt_pid = app.ctr.Kd_jnt_pid_1;
        end
    else
        app.ctr.tcyc = app.ctr.tcyc_0;
        if app.ctr.algo == 1
        else
            if app.kin.n == 7
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1); ...
                                      app.ctr.Kp_jnt_pid_0(2) app.ctr.Ki_jnt_pid_0(2) app.ctr.Kd_jnt_pid_0(2); ...
                                      app.ctr.Kp_jnt_pid_0(3) app.ctr.Ki_jnt_pid_0(3) app.ctr.Kd_jnt_pid_0(3); ...
                                      app.ctr.Kp_jnt_pid_0(4) app.ctr.Ki_jnt_pid_0(4) app.ctr.Kd_jnt_pid_0(4); ...
                                      app.ctr.Kp_jnt_pid_0(5) app.ctr.Ki_jnt_pid_0(5) app.ctr.Kd_jnt_pid_0(5); ...
                                      app.ctr.Kp_jnt_pid_0(6) app.ctr.Ki_jnt_pid_0(6) app.ctr.Kd_jnt_pid_0(6); ...
                                      app.ctr.Kp_jnt_pid_0(7) app.ctr.Ki_jnt_pid_0(7) app.ctr.Kd_jnt_pid_0(7)};
            elseif app.kin.n == 6
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1); ...
                                      app.ctr.Kp_jnt_pid_0(2) app.ctr.Ki_jnt_pid_0(2) app.ctr.Kd_jnt_pid_0(2); ...
                                      app.ctr.Kp_jnt_pid_0(3) app.ctr.Ki_jnt_pid_0(3) app.ctr.Kd_jnt_pid_0(3); ...
                                      app.ctr.Kp_jnt_pid_0(4) app.ctr.Ki_jnt_pid_0(4) app.ctr.Kd_jnt_pid_0(4); ...
                                      app.ctr.Kp_jnt_pid_0(5) app.ctr.Ki_jnt_pid_0(5) app.ctr.Kd_jnt_pid_0(5); ...
                                      app.ctr.Kp_jnt_pid_0(6) app.ctr.Ki_jnt_pid_0(6) app.ctr.Kd_jnt_pid_0(6)};
            elseif app.kin.n == 5
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1); ...
                                      app.ctr.Kp_jnt_pid_0(2) app.ctr.Ki_jnt_pid_0(2) app.ctr.Kd_jnt_pid_0(2); ...
                                      app.ctr.Kp_jnt_pid_0(3) app.ctr.Ki_jnt_pid_0(3) app.ctr.Kd_jnt_pid_0(3); ...
                                      app.ctr.Kp_jnt_pid_0(4) app.ctr.Ki_jnt_pid_0(4) app.ctr.Kd_jnt_pid_0(4); ...
                                      app.ctr.Kp_jnt_pid_0(5) app.ctr.Ki_jnt_pid_0(5) app.ctr.Kd_jnt_pid_0(5)};
            elseif app.kin.n == 4
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1); ...
                                      app.ctr.Kp_jnt_pid_0(2) app.ctr.Ki_jnt_pid_0(2) app.ctr.Kd_jnt_pid_0(2); ...
                                      app.ctr.Kp_jnt_pid_0(3) app.ctr.Ki_jnt_pid_0(3) app.ctr.Kd_jnt_pid_0(3); ...
                                      app.ctr.Kp_jnt_pid_0(4) app.ctr.Ki_jnt_pid_0(4) app.ctr.Kd_jnt_pid_0(4)};
            elseif app.kin.n == 3
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1); ...
                                      app.ctr.Kp_jnt_pid_0(2) app.ctr.Ki_jnt_pid_0(2) app.ctr.Kd_jnt_pid_0(2); ...
                                      app.ctr.Kp_jnt_pid_0(3) app.ctr.Ki_jnt_pid_0(3) app.ctr.Kd_jnt_pid_0(3)};
            elseif app.kin.n == 2
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1); ...
                                      app.ctr.Kp_jnt_pid_0(2) app.ctr.Ki_jnt_pid_0(2) app.ctr.Kd_jnt_pid_0(2)};
            else
                app.GainTable.Data = {app.ctr.Kp_jnt_pid_0(1) app.ctr.Ki_jnt_pid_0(1) app.ctr.Kd_jnt_pid_0(1)};
            end
            app.ctr.Kp_jnt_pid = app.ctr.Kp_jnt_pid_0;
            app.ctr.Ki_jnt_pid = app.ctr.Ki_jnt_pid_0;
            app.ctr.Kd_jnt_pid = app.ctr.Kd_jnt_pid_0;
        end
    end

    app.CycleEditField.Value = app.ctr.tcyc;
    app.CompGravityCheckBox.Value = app.ctr.comp_grv;
    
    if app.kin.n == 7
        app.GainTable.RowName = {'1'; '2'; '3'; '4'; '5'; '6'; '7'};
    elseif app.kin.n == 6
        app.GainTable.RowName = {'1'; '2'; '3'; '4'; '5'; '6'};
    elseif app.kin.n == 5
        app.GainTable.RowName = {'1'; '2'; '3'; '4'; '5'};
    elseif app.kin.n == 4
        app.GainTable.RowName = {'1'; '2'; '3'; '4'};
    elseif app.kin.n == 3
        app.GainTable.RowName = {'1'; '2'; '3'};
    elseif app.kin.n == 2
        app.GainTable.RowName = {'1'; '2'};
    else
        app.GainTable.RowName = {'1'};
    end

    app.GainTable.ColumnEditable = logical([1 1 1]);
    app.GainTable.ColumnWidth = {65,65,65};

end