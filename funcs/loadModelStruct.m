function loadModelStruct(app)
    if ~isfield(app.ms, 'renderer')
        app.ms.renderer = RobotRenderer(app.UIAxes);
    end
    app.ms.renderer.loadMeshes(app.robot_model, app.high_quality, app.ee_att, app.coord_frame_on, app.ghost_on, app.line_on, app.task_mode, app.running_flag, app.trj_profile);
end
