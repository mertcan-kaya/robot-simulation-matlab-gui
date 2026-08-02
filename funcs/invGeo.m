function qDes = invGeo(app,RGoal,tGoal,q0)
    % invGeo is now a wrapper that extracts parameters from the app 
    % and passes them to the decoupled invGeoCore mathematical solver.

    % Pack the configuration required by Inverse Kinematics
    invGeoConfig.inv_geo_type = app.controller.model.inv_geo_type;
    invGeoConfig.robot_model = app.controller.model.robot_model;
    invGeoConfig.TI_0 = app.controller.model.TI_0;
    invGeoConfig.inv_geo_trn = app.controller.model.inv_geo_trn;
    invGeoConfig.kp_inv = app.controller.model.kp_inv;
    invGeoConfig.kr_inv = app.controller.model.kr_inv;
    invGeoConfig.kp_trn = app.controller.model.kp_trn;
    invGeoConfig.kr_trn = app.controller.model.kr_trn;

    % Call the decoupled core function
    qDes = invGeoCore(invGeoConfig, app.controller.model.kin, RGoal, tGoal, q0);

end