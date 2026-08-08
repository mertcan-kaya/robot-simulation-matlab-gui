function plotAxesOutApp(app)
    if ~isfield(app.ms, 'renderer')
        return;
    end
    
    % Package state cleanly
    state.running_flag = app.running_flag;
    state.TI_0 = app.TI_0;
    state.kin = app.kin;
    state.act = app.act;
    state.des = app.des;
    state.ini = app.ini;
    state.fin = app.fin;
    state.task_mode = app.task_mode;
    state.ee_att = app.ee_att;
    state.robot_model = app.robot_model;
    state.line_on = app.line_on;
    state.ghost_on = app.ghost_on;
    state.coord_frame_on = app.coord_frame_on;
    state.qEDes = app.qEDes;
    state.qEIni = app.qEIni;
    state.trj_profile = app.trj_profile;
    
    app.ms.renderer.updateView(state);
end
